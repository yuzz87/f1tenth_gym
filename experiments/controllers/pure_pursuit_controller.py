# 既存のPure Pursuit実装を読み込む
from examples.waypoint_follow import PurePursuitPlanner

# 共通コントローラの基底クラスを読み込む
from .base_controller import BaseController


class PurePursuitController(BaseController):
    """既存のPurePursuitPlannerを共通コントローラ形式へ適合させるクラス。"""

    def __init__(self, conf, car_params):
        # 設定全体を保存
        self.conf = conf

        # Pure Pursuitに関係する設定だけを取り出す
        self.controller_conf = conf.controller

        # 重心から前輪までの距離と、
        # 重心から後輪までの距離を足してホイールベースを求める
        wheelbase = car_params["lf"] + car_params["lr"]

        # 既存のPure Pursuitプランナーを初期化する
        self.planner = PurePursuitPlanner(conf, wheelbase)

        # 読み込まれた目標経路を外部からも参照できるようにする
        self.waypoints = self.planner.waypoints

    def plan(self, obs):
        # 0番目の車両の現在位置と姿勢を取り出し、
        # Pure Pursuitで目標速度・目標操舵角を計算する
        speed, steer = self.planner.plan(
            obs["poses_x"][0],       # 現在のx座標
            obs["poses_y"][0],       # 現在のy座標
            obs["poses_theta"][0],   # 現在の車両姿勢[rad]
            self.controller_conf["tlad"],   # 先読み距離
            self.controller_conf["vgain"],  # 速度ゲイン
        )

        # NumPy型などではなく、Python標準のfloatで返す
        return float(speed), float(steer)

    def render_waypoints(self, env_renderer):
        # 目標経路のwaypointを画面に描画する
        self.planner.render_waypoints(env_renderer)
