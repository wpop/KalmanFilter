"""
Эталонная реализация линейного фильтра Калмана для наземного робота (ровера).

Состояние:      x = [x, y, vx, vy]^T   (положение в метрах и скорость в м/с в локальной системе ENU)
Управление:     u = [ax, ay]^T         (ускорение, измеренное IMU и повёрнутое в систему ENU)
Измерения:      GPS-позиция (+ опционально скорость из RMC)

Архитектура (SOLID):
    * S — каждый класс отвечает за одно: модель движения, модель измерения, сам фильтр.
    * O — новые модели движения / измерений добавляются наследованием, фильтр не меняется.
    * L — любая реализация MotionModel / MeasurementModel взаимозаменяема.
    * I — интерфейсы узкие: у модели движения три метода, у модели измерения — два.
    * D — KalmanFilter зависит от абстракций (ABC), а не от конкретных матриц.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


# --------------------------------------------------------------------------- #
#  Модели движения
# --------------------------------------------------------------------------- #
class MotionModel(ABC):
    """Абстракция «как система эволюционирует во времени»."""

    @property
    @abstractmethod
    def state_dim(self) -> int:
        """Размерность вектора состояния."""

    @abstractmethod
    def transition(self, dt: float) -> np.ndarray:
        """Матрица перехода F(dt)."""

    @abstractmethod
    def control(self, dt: float) -> np.ndarray:
        """Матрица управления B(dt)."""

    @abstractmethod
    def process_noise(self, dt: float) -> np.ndarray:
        """Ковариация шума процесса Q(dt)."""


class ConstantAccelerationModel2D(MotionModel):
    """
    Движение на плоскости: ускорение приходит от IMU как управляющий вход,
    а его неточность описывается шумом процесса.

        x'  = x  + vx*dt + 0.5*ax*dt^2
        vx' = vx + ax*dt
    """

    def __init__(self, sigma_acc: float) -> None:
        if sigma_acc <= 0:
            raise ValueError("sigma_acc должна быть положительной")
        self._sigma_acc = sigma_acc  # СКО шума акселерометра, м/с^2

    @property
    def state_dim(self) -> int:
        return 4

    def transition(self, dt: float) -> np.ndarray:
        # В F стоит именно dt (а не скорость): положение += скорость * dt
        f = np.eye(4)
        f[0, 2] = dt
        f[1, 3] = dt
        return f

    def control(self, dt: float) -> np.ndarray:
        # Ускорение вносит вклад 0.5*dt^2 в положение и dt в скорость
        half_dt2 = 0.5 * dt * dt
        return np.array([
            [half_dt2, 0.0],
            [0.0, half_dt2],
            [dt, 0.0],
            [0.0, dt],
        ])

    def process_noise(self, dt: float) -> np.ndarray:
        # Шум ускорения заходит в состояние ровно через B, поэтому Q = sigma^2 * B * B^T.
        # Матрица получается симметричной и неотрицательно определённой по построению.
        b = self.control(dt)
        return (self._sigma_acc ** 2) * (b @ b.T)


# --------------------------------------------------------------------------- #
#  Модели измерений
# --------------------------------------------------------------------------- #
class MeasurementModel(ABC):
    """Абстракция «что и с какой точностью мы наблюдаем»."""

    @abstractmethod
    def observation_matrix(self) -> np.ndarray:
        """Матрица наблюдения H (размер m x n)."""

    @abstractmethod
    def noise_covariance(self) -> np.ndarray:
        """Ковариация шума измерения R (размер m x m — согласована с H!)."""


class PositionMeasurement(MeasurementModel):
    """GPS даёт только положение (x, y)."""

    # Грубая оценка: СКО позиции ≈ HDOP * UERE. Значение UERE — допущение, его нужно
    # откалибровать по реальным данным (для обычного смартфона порядка 3–5 м).
    DEFAULT_UERE_M = 3.0

    def __init__(self, sigma_pos: float) -> None:
        if sigma_pos <= 0:
            raise ValueError("sigma_pos должна быть положительной")
        self._sigma_pos = sigma_pos

    @classmethod
    def from_hdop(cls, hdop: float, uere: float = DEFAULT_UERE_M) -> "PositionMeasurement":
        """Строим модель измерения из HDOP, который уже парсится из GSA."""
        return cls(sigma_pos=max(hdop, 1.0) * uere)

    def observation_matrix(self) -> np.ndarray:
        return np.eye(2, 4)

    def noise_covariance(self) -> np.ndarray:
        return np.eye(2) * self._sigma_pos ** 2


class PositionVelocityMeasurement(MeasurementModel):
    """GPS даёт и положение, и скорость (например, из RMC, разложенная по осям)."""

    def __init__(self, sigma_pos: float, sigma_vel: float) -> None:
        if sigma_pos <= 0 or sigma_vel <= 0:
            raise ValueError("СКО должны быть положительными")
        self._sigma_pos = sigma_pos
        self._sigma_vel = sigma_vel

    def observation_matrix(self) -> np.ndarray:
        return np.eye(4)

    def noise_covariance(self) -> np.ndarray:
        return np.diag([self._sigma_pos ** 2, self._sigma_pos ** 2,
                        self._sigma_vel ** 2, self._sigma_vel ** 2])


# --------------------------------------------------------------------------- #
#  Результаты и сам фильтр
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class UpdateReport:
    """Диагностика шага коррекции — пригодится для отсева выбросов GPS."""

    innovation: np.ndarray  # невязка y = z - H x
    nis: float              # нормированный квадрат невязки y^T S^-1 y (≈ хи-квадрат)


class KalmanFilter:
    """Линейный фильтр Калмана: цикл predict -> update."""

    def __init__(self, motion_model: MotionModel,
                 initial_state: np.ndarray,
                 initial_covariance: np.ndarray) -> None:
        n = motion_model.state_dim
        x0 = np.asarray(initial_state, dtype=float).reshape(n, 1)
        p0 = np.asarray(initial_covariance, dtype=float)
        if p0.shape != (n, n):
            raise ValueError(f"P0 должна быть {n}x{n}")
        self._model = motion_model
        self._x = x0
        self._p = p0

    @property
    def state(self) -> np.ndarray:
        return self._x.ravel().copy()

    @property
    def covariance(self) -> np.ndarray:
        return self._p.copy()

    def predict(self, dt: float, control: np.ndarray | None = None) -> None:
        """Прогноз: куда система пришла за время dt, и насколько мы стали менее уверены."""
        if dt <= 0:
            raise ValueError("dt должен быть положительным")
        f = self._model.transition(dt)
        q = self._model.process_noise(dt)

        self._x = f @ self._x
        if control is not None:
            b = self._model.control(dt)
            self._x += b @ np.asarray(control, dtype=float).reshape(-1, 1)

        self._p = f @ self._p @ f.T + q

    def update(self, measurement: np.ndarray, model: MeasurementModel) -> UpdateReport:
        """Коррекция: смешиваем прогноз с измерением пропорционально доверию к каждому."""
        h = model.observation_matrix()
        r = model.noise_covariance()
        z = np.asarray(measurement, dtype=float).reshape(-1, 1)
        if h.shape[0] != z.shape[0] or r.shape != (h.shape[0], h.shape[0]):
            raise ValueError("Размерности z, H и R не согласованы")

        y = z - h @ self._x                     # невязка
        s = h @ self._p @ h.T + r               # ковариация невязки
        # Коэффициент усиления K = P H^T S^-1; решаем систему вместо явного обращения матрицы
        k = np.linalg.solve(s, h @ self._p).T

        self._x = self._x + k @ y

        # Форма Джозефа: сохраняет симметрию и положительную определённость P при ошибках округления
        i_kh = np.eye(self._p.shape[0]) - k @ h
        self._p = i_kh @ self._p @ i_kh.T + k @ r @ k.T
        self._p = 0.5 * (self._p + self._p.T)

        nis = (y.T @ np.linalg.solve(s, y)).item()  # результат 1x1 -> обычное число
        return UpdateReport(innovation=y.ravel(), nis=nis)


# --------------------------------------------------------------------------- #
#  Демонстрация на симуляции: проверяем, что фильтр действительно лучше «сырого» GPS
# --------------------------------------------------------------------------- #
def _run_demo(seed: int = 7) -> None:
    rng = np.random.default_rng(seed)

    dt_imu = 0.1          # IMU работает на 10 Гц
    gps_every = 10        # GPS — раз в секунду
    steps = 600           # 60 секунд движения
    sigma_gps = 3.0       # реальная ошибка GPS, м
    sigma_acc = 0.2       # реальная ошибка акселерометра, м/с^2

    # Истинная траектория: разгон, поворот (ускорение по y), торможение
    true_acc = np.zeros((steps, 2))
    true_acc[:100, 0] = 0.5
    true_acc[200:300, 1] = 0.4
    true_acc[400:500, 0] = -0.4

    true_state = np.zeros((steps, 4))
    s = np.zeros(4)
    for i in range(steps):
        a = true_acc[i]
        s = np.array([s[0] + s[2] * dt_imu + 0.5 * a[0] * dt_imu ** 2,
                      s[1] + s[3] * dt_imu + 0.5 * a[1] * dt_imu ** 2,
                      s[2] + a[0] * dt_imu,
                      s[3] + a[1] * dt_imu])
        true_state[i] = s

    # Фильтр стартует «вслепую»: позиция с ошибкой, скорость неизвестна
    kf = KalmanFilter(
        motion_model=ConstantAccelerationModel2D(sigma_acc=sigma_acc),
        initial_state=np.array([5.0, -5.0, 0.0, 0.0]),
        initial_covariance=np.diag([25.0, 25.0, 4.0, 4.0]),
    )
    gps_model = PositionMeasurement(sigma_pos=sigma_gps)

    err_kf, err_gps = [], []
    for i in range(steps):
        imu = true_acc[i] + rng.normal(0.0, sigma_acc, 2)
        kf.predict(dt_imu, control=imu)

        if (i + 1) % gps_every == 0:
            gps = true_state[i, :2] + rng.normal(0.0, sigma_gps, 2)
            kf.update(gps, gps_model)
            err_gps.append(np.linalg.norm(gps - true_state[i, :2]))
            err_kf.append(np.linalg.norm(kf.state[:2] - true_state[i, :2]))

    # Пропускаем первые 10 эпох: фильтр ещё «сходится» из неверного начального состояния
    rmse = lambda e: float(np.sqrt(np.mean(np.square(e[10:]))))
    print(f"RMSE сырого GPS : {rmse(err_gps):.2f} м")
    print(f"RMSE фильтра    : {rmse(err_kf):.2f} м")


if __name__ == "__main__":
    _run_demo()
