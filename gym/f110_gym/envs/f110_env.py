# MIT License

# Copyright (c) 2020 Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
コードの簡易解説:
このファイルは F1TENTH Gym の Gym 環境クラス `F110Env` を定義する。
主な役割は、車両パラメータとマップを読み込み、シミュレータを初期化し、
`reset()` で初期姿勢へ戻し、`step()` で1ステップ分だけ車両状態を進め、
`render()` で現在の状態を描画すること。

制御器側からは `env.step(np.array([[steer, speed]]))` の形で入力する。
ここで `steer` は操舵角指令、`speed` は速度指令を表す。

"""

# Gym関連のインポート
import os
import time

import gym

# その他の標準/外部ライブラリ
import numpy as np

# OpenGL/描画関連
import pyglet
from gym import error, spaces, utils
from gym.utils import seeding

# シミュレータ本体と積分器
from f110_gym.envs.base_classes import Integrator, Simulator

pyglet.options["debug_gl"] = False
from pyglet import gl

# 定数

# 描画サイズ
VIDEO_W = 600
VIDEO_H = 400
WINDOW_W = 1000
WINDOW_H = 800


class F110Env(gym.Env):
    """
    F1TENTH用のOpenAI Gym環境。

    基本的には `gym.make('f110_gym:f110-v0', **kwargs)` を呼び出して初期化する。

    引数:
        kwargs:
            seed (int, default=12345): 乱数状態を再現するためのシード値。

            map (str, default='vegas'): 環境で使うマップ名。組み込みでは
                'berlin', 'vegas', 'skirk' などが使える。独自マップを使う場合は、
                yamlファイルへの絶対パスを文字列で渡すこともできる。

            map_ext (str, default='png'): マップ画像の拡張子。例: 'png', 'pgm'。

            params (dict): 車両パラメータの辞書。
            mu: 路面摩擦係数。
            C_Sf: 前輪のコーナリングスティフネス係数。
            C_Sr: 後輪のコーナリングスティフネス係数。
            lf: 重心から前輪軸までの距離。
            lr: 重心から後輪軸までの距離。
            h: 重心高さ。
            m: 車両全体の質量。
            I: z軸まわりの慣性モーメント。
            s_min: 最小操舵角制約。
            s_max: 最大操舵角制約。
            sv_min: 最小操舵角速度制約。
            sv_max: 最大操舵角速度制約。
            v_switch: 加速制約の切り替え速度。
            a_max: 最大前後加速度。
            v_min: 最小前後速度。
            v_max: 最大前後速度。
            width: 車幅[m]。
            length: 車長[m]。

            num_agents (int, default=2): 環境内の車両台数。

            timestep (float, default=0.01): 物理シミュレーションの時間刻み。

            ego_idx (int, default=0): 自車として扱う車両のインデックス。

            lidar_dist (float, default=0): 後輪軸からLiDARまでの距離。
    """

    metadata = {"render.modes": ["human", "human_fast"]}

    # 描画用の共有状態
    renderer = None
    current_obs = None
    render_callbacks = []

    def __init__(self, **kwargs):
        # kwargsから設定値を取り出す。
        try:
            self.seed = kwargs["seed"]
        except:
            self.seed = 12345
        try:
            self.map_name = kwargs["map"]
            # 組み込みマップ名が渡された場合は、対応するyamlファイルを使う。
            if self.map_name == "berlin":
                self.map_path = (
                    os.path.dirname(os.path.abspath(__file__)) + "/maps/berlin.yaml"
                )
            elif self.map_name == "skirk":
                self.map_path = (
                    os.path.dirname(os.path.abspath(__file__)) + "/maps/skirk.yaml"
                )
            elif self.map_name == "levine":
                self.map_path = (
                    os.path.dirname(os.path.abspath(__file__)) + "/maps/levine.yaml"
                )
            else:
                self.map_path = self.map_name + ".yaml"
        except:
            self.map_path = (
                os.path.dirname(os.path.abspath(__file__)) + "/maps/vegas.yaml"
            )

        try:
            self.map_ext = kwargs["map_ext"]
        except:
            self.map_ext = ".png"

        try:
            self.params = kwargs["params"]
        except:
            self.params = {
                "mu": 1.0489,
                "C_Sf": 4.718,
                "C_Sr": 5.4562,
                "lf": 0.15875,
                "lr": 0.17145,
                "h": 0.074,
                "m": 3.74,
                "I": 0.04712,
                "s_min": -0.4189,
                "s_max": 0.4189,
                "sv_min": -3.2,
                "sv_max": 3.2,
                "v_switch": 7.319,
                "a_max": 9.51,
                "v_min": -5.0,
                "v_max": 20.0,
                "width": 0.31,
                "length": 0.58,
            }

        # シミュレーション設定
        try:
            self.num_agents = kwargs["num_agents"]
        except:
            self.num_agents = 2

        try:
            self.timestep = kwargs["timestep"]
        except:
            self.timestep = 0.01

        # デフォルトの自車インデックス
        try:
            self.ego_idx = kwargs["ego_idx"]
        except:
            self.ego_idx = 0

        # デフォルトの積分器
        try:
            self.integrator = kwargs["integrator"]
        except:
            self.integrator = Integrator.RK4

        # デフォルトのLiDAR位置
        try:
            self.lidar_dist = kwargs["lidar_dist"]
        except:
            self.lidar_dist = 0.0

        # スタート近傍とみなす半径
        self.start_thresh = 0.5  # 10cm

        # 環境状態
        self.poses_x = []
        self.poses_y = []
        self.poses_theta = []
        self.collisions = np.zeros((self.num_agents,))
        # TODO: collision_idx はまだ使われていない。
        # self.collision_idx = -1 * np.ones((self.num_agents, ))

        # 周回完了判定用
        self.near_start = True
        self.num_toggles = 0

        # レース情報
        self.lap_times = np.zeros((self.num_agents,))
        self.lap_counts = np.zeros((self.num_agents,))
        self.current_time = 0.0

        # フィニッシュライン判定用
        self.num_toggles = 0
        self.near_start = True
        self.near_starts = np.array([True] * self.num_agents)
        self.toggle_list = np.zeros((self.num_agents,))
        self.start_xs = np.zeros((self.num_agents,))
        self.start_ys = np.zeros((self.num_agents,))
        self.start_thetas = np.zeros((self.num_agents,))
        self.start_rot = np.eye(2)

        # Simulatorを初期化してマップを読み込む。
        self.sim = Simulator(
            self.params,
            self.num_agents,
            self.seed,
            time_step=self.timestep,
            integrator=self.integrator,
            lidar_dist=self.lidar_dist,
        )
        self.sim.set_map(self.map_path, self.map_ext)

        # 描画用に保持する観測
        self.render_obs = None

    def __del__(self):
        """
        終了時処理。
        現状では特別な後処理は行わない。
        """
        pass

    def _check_done(self):
        """
        現在の走行エピソードが終了したかを判定する。

        引数:
            なし

        戻り値:
            done (bool): エピソードが終了したかどうか。
            toggle_list (list[int]): 各車両がフィニッシュ領域を通過したかどうか。
        """

        # もともとは2台走行を想定した判定。
        # TODO: 将来的にはs座標ベースの判定へ変更する候補。
        left_t = 2
        right_t = 2

        poses_x = np.array(self.poses_x) - self.start_xs
        poses_y = np.array(self.poses_y) - self.start_ys
        delta_pt = np.dot(self.start_rot, np.stack((poses_x, poses_y), axis=0))
        temp_y = delta_pt[1, :]
        idx1 = temp_y > left_t
        idx2 = temp_y < -right_t
        temp_y[idx1] -= left_t
        temp_y[idx2] = -right_t - temp_y[idx2]
        temp_y[np.invert(np.logical_or(idx1, idx2))] = 0

        dist2 = delta_pt[0, :] ** 2 + temp_y**2
        closes = dist2 <= 0.1
        for i in range(self.num_agents):
            if closes[i] and not self.near_starts[i]:
                self.near_starts[i] = True
                self.toggle_list[i] += 1
            elif not closes[i] and self.near_starts[i]:
                self.near_starts[i] = False
                self.toggle_list[i] += 1
            self.lap_counts[i] = self.toggle_list[i] // 2
            if self.toggle_list[i] < 4:
                self.lap_times[i] = self.current_time

        done = (self.collisions[self.ego_idx]) or np.all(self.toggle_list >= 4)

        return bool(done), self.toggle_list >= 4

    def _update_state(self, obs_dict):
        """
        観測値に合わせて環境側の状態を更新する。

        引数:
            obs_dict (dict): 観測値の辞書。

        戻り値:
            なし
        """
        self.poses_x = obs_dict["poses_x"]
        self.poses_y = obs_dict["poses_y"]
        self.poses_theta = obs_dict["poses_theta"]
        self.collisions = obs_dict["collisions"]

    def step(self, action):
        """
        Gym環境を1ステップ進める。

        引数:
            action (np.ndarray(num_agents, 2)): 各車両への入力。
                入力の順番は `[steer, speed]`。

        戻り値:
            obs (dict): 現在ステップの観測値。
            reward (float): ステップ報酬。現状では物理時間刻み `timestep`。
            done (bool): シミュレーションが終了したかどうか。
            info (dict): 追加情報の辞書。
        """

        # シミュレータ本体を1ステップ進める。
        obs = self.sim.step(action)
        obs["lap_times"] = self.lap_times
        obs["lap_counts"] = self.lap_counts

        F110Env.current_obs = obs

        self.render_obs = {
            "ego_idx": obs["ego_idx"],
            "poses_x": obs["poses_x"],
            "poses_y": obs["poses_y"],
            "poses_theta": obs["poses_theta"],
            "lap_times": obs["lap_times"],
            "lap_counts": obs["lap_counts"],
        }

        # 時刻と報酬を更新する。
        reward = self.timestep
        self.current_time = self.current_time + self.timestep

        # 環境が保持する状態を更新する。
        self._update_state(obs)

        # 終了判定を行う。
        done, toggle_list = self._check_done()
        info = {"checkpoint_done": toggle_list}

        return obs, reward, done, info

    def reset(self, poses):
        """
        指定した姿勢でGym環境をリセットする。

        引数:
            poses (np.ndarray (num_agents, 3)): 各車両の初期姿勢 `[x, y, theta]`。

        戻り値:
            obs (dict): リセット直後の観測値。
            reward (float): ステップ報酬。現状では物理時間刻み `timestep`。
            done (bool): シミュレーションが終了したかどうか。
            info (dict): 追加情報の辞書。
        """
        # カウンタと状態変数をリセットする。
        self.current_time = 0.0
        self.collisions = np.zeros((self.num_agents,))
        self.num_toggles = 0
        self.near_start = True
        self.near_starts = np.array([True] * self.num_agents)
        self.toggle_list = np.zeros((self.num_agents,))

        # リセット後の初期状態を保存する。
        self.start_xs = poses[:, 0]
        self.start_ys = poses[:, 1]
        self.start_thetas = poses[:, 2]
        self.start_rot = np.array(
            [
                [
                    np.cos(-self.start_thetas[self.ego_idx]),
                    -np.sin(-self.start_thetas[self.ego_idx]),
                ],
                [
                    np.sin(-self.start_thetas[self.ego_idx]),
                    np.cos(-self.start_thetas[self.ego_idx]),
                ],
            ]
        )

        # シミュレータ本体をリセットする。
        self.sim.reset(poses)

        # 入力なしの状態で最初の観測を取得する。
        action = np.zeros((self.num_agents, 2))
        obs, reward, done, info = self.step(action)

        self.render_obs = {
            "ego_idx": obs["ego_idx"],
            "poses_x": obs["poses_x"],
            "poses_y": obs["poses_y"],
            "poses_theta": obs["poses_theta"],
            "lap_times": obs["lap_times"],
            "lap_counts": obs["lap_counts"],
        }

        return obs, reward, done, info

    def update_map(self, map_path, map_ext):
        """
        シミュレーションで使うマップを更新する。

        引数:
            map_path (str): マップyamlファイルへの絶対パス。
            map_ext (str): マップ画像ファイルの拡張子。

        戻り値:
            なし
        """
        self.sim.set_map(map_path, map_ext)

    def update_params(self, params, index=-1):
        """
        シミュレーションで使う車両パラメータを更新する。

        引数:
            params (dict): 車両パラメータの辞書。
            index (int, default=-1): 0以上なら指定した車両だけを更新する。

        戻り値:
            なし
        """
        self.sim.update_params(params, agent_idx=index)

    def add_render_callback(self, callback_func):
        """
        描画時に呼び出す追加の描画関数を登録する。

        引数:
            callback_func (function (EnvRenderer) -> None): render()中に呼び出す独自関数。
        """

        F110Env.render_callbacks.append(callback_func)

    def render(self, mode="human"):
        """
        pygletを使って環境を描画する。
        マウスホイールでズーム、ドラッグで画面移動ができる。
        車両、マップ、現在のfps、レース情報を画面に表示する。

        引数:
            mode (str, default='human'): 描画モード。
                'human': 実時間に近くなるように少し待ちながら描画する。
                'human_fast': できるだけ速く描画する。

        戻り値:
            なし
        """
        assert mode in ["human", "human_fast"]

        if F110Env.renderer is None:
            # 初回呼び出し時に描画器を初期化する。
            from f110_gym.envs.rendering import EnvRenderer

            F110Env.renderer = EnvRenderer(WINDOW_W, WINDOW_H)
            F110Env.renderer.update_map(self.map_name, self.map_ext)

        F110Env.renderer.update_obs(self.render_obs)

        for render_callback in F110Env.render_callbacks:
            render_callback(F110Env.renderer)

        F110Env.renderer.dispatch_events()
        F110Env.renderer.on_draw()
        F110Env.renderer.flip()
        if mode == "human":
            time.sleep(0.005)
        elif mode == "human_fast":
            pass
