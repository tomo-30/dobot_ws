#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Dobot Nova 5 GUI (based on demo.py)

・demo.py の adderClient をそのまま利用
・4パターンの移動：
   ① MovJ（絶対：関節補間で手先姿勢へ）
   ② MovL（絶対：手先直線）※デフォルト
   ③ RelMovJ（相対：関節角オフセット）
   ④ RelMovL（相対：手先オフセット）
・速度関連は demo.py の set_parameter_L(SF,AccL,SpeedL,CP) をそのまま呼び出し
・ホームボタン：node.point("MovJ", -400, 100, 260, -180, 0, -90)
・終了ボタン：Disable 等を実行してからGUI終了
"""

import threading
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
# demo.py のクラスを再利用（main は実行されません）
from dobot_demo.demo import adderClient

# ホーム姿勢（MovJ 絶対指定）
HOME_POSE = (-400.0, 100.0, 260.0, -180.0, 0.0, -90.0)


class NovaGUI:
    """
    4パターンの移動をGUIで指定して実行する：
      ① MovJ（絶対：関節補間で手先姿勢へ）
      ② MovL（絶対：手先直線）←デフォルト
      ③ RelMovJ（相対：関節角オフセット）
      ④ RelMovL（相対：手先オフセット）
    """
    def __init__(self, node: adderClient):
        self.node = node

        self.root = tk.Tk()
        self.root.title('Dobot Nova5 GUI (based on demo.py)')
        self.root.geometry('720x560')

        pad = {'padx': 8, 'pady': 6}

        # --- 上段：基本操作 ---
        top = ttk.Frame(self.root); top.pack(fill='x', **pad)
        ttk.Button(top, text='Enable',  command=self.on_enable).pack(side='left', padx=4)
        ttk.Button(top, text='Disable', command=self.on_disable).pack(side='left', padx=4)
        ttk.Button(top, text='Get Pose → ABS欄', command=self.on_get_pose).pack(side='left', padx=12)
        ttk.Button(top, text='Home', command=self.on_home).pack(side='left', padx=4)
        ttk.Button(top, text='Exit', command=self.on_exit).pack(side='left', padx=4)

        # --- 速度パラメータ ---
        sp = ttk.LabelFrame(self.root, text='速度パラメータ (set_parameter_L: SpeedFactor, AccL, SpeedL, CP)')
        sp.pack(fill='x', **pad)
        self.v_sf = tk.IntVar(value=10)
        self.v_al = tk.IntVar(value=50)
        self.v_sl = tk.IntVar(value=50)
        self.v_cp = tk.IntVar(value=100)
        _row = 0
        ttk.Label(sp, text='SpeedFactor(1-100)').grid(row=_row, column=0, sticky='e')
        ttk.Entry(sp, width=6, textvariable=self.v_sf).grid(row=_row, column=1, sticky='w')
        ttk.Label(sp, text='AccL(1-100)').grid(row=_row, column=2, sticky='e')
        ttk.Entry(sp, width=6, textvariable=self.v_al).grid(row=_row, column=3, sticky='w')
        ttk.Label(sp, text='SpeedL(1-100)').grid(row=_row, column=4, sticky='e')
        ttk.Entry(sp, width=6, textvariable=self.v_sl).grid(row=_row, column=5, sticky='w')
        ttk.Label(sp, text='CP(1-100)').grid(row=_row, column=6, sticky='e')
        ttk.Entry(sp, width=6, textvariable=self.v_cp).grid(row=_row, column=7, sticky='w')
        ttk.Button(sp, text='Apply', command=self.on_apply_speed).grid(row=_row, column=8, padx=12)

        # --- 動作モード選択 ---
        md = ttk.LabelFrame(self.root, text='動き方（デフォルト②MovL：絶対・直線）')
        md.pack(fill='x', **pad)
        self.mode = tk.StringVar(value='MovL')  # ②をデフォルト
        modes = [
            ('① 絶対 MovJ（関節補間で手先姿勢へ）', 'MovJ'),
            ('② 絶対 MovL（手先直線で姿勢へ）',    'MovL'),
            ('③ 相対 RelMovJ（関節角オフセット）', 'RelMovJ'),
            ('④ 相対 RelMovL（手先オフセット）',   'RelMovL'),
        ]
        for i, (label, val) in enumerate(modes):
            ttk.Radiobutton(md, text=label, value=val, variable=self.mode,
                            command=self._on_mode_change).grid(row=0, column=i, sticky='w', padx=6)

        # --- 入力欄：絶対姿勢（①②） ---
        self.grp_abs = ttk.LabelFrame(self.root, text='絶対姿勢（①MovJ / ②MovL）: [mm, deg]')
        self.grp_abs.pack(fill='x', **pad)
        self.ax = tk.DoubleVar(value=HOME_POSE[0])
        self.ay = tk.DoubleVar(value=HOME_POSE[1])
        self.az = tk.DoubleVar(value=HOME_POSE[2])
        self.arx = tk.DoubleVar(value=HOME_POSE[3])
        self.ary = tk.DoubleVar(value=HOME_POSE[4])
        self.arz = tk.DoubleVar(value=HOME_POSE[5])
        self._row_inputs(self.grp_abs, ['X','Y','Z','Rx','Ry','Rz'],
                         [self.ax,self.ay,self.az,self.arx,self.ary,self.arz])

        # --- 入力欄：相対（③ 関節角[deg]） ---
        self.grp_jrel = ttk.LabelFrame(self.root, text='相対（③RelMovJ）: ΔJ1..ΔJ6 [deg]')
        self.grp_jrel.pack(fill='x', **pad)
        self.dj1 = tk.DoubleVar(value=0.0)
        self.dj2 = tk.DoubleVar(value=0.0)
        self.dj3 = tk.DoubleVar(value=0.0)
        self.dj4 = tk.DoubleVar(value=0.0)
        self.dj5 = tk.DoubleVar(value=20.0)
        self.dj6 = tk.DoubleVar(value=0.0)
        self._row_inputs(self.grp_jrel, ['ΔJ1','ΔJ2','ΔJ3','ΔJ4','ΔJ5','ΔJ6'],
                         [self.dj1,self.dj2,self.dj3,self.dj4,self.dj5,self.dj6])

        # --- 入力欄：相対（④ 手先[mm, deg]） ---
        self.grp_trel = ttk.LabelFrame(self.root, text='相対（④RelMovL）: ΔX ΔY ΔZ ΔRx ΔRy ΔRz [mm, deg]')
        self.grp_trel.pack(fill='x', **pad)
        self.dx = tk.DoubleVar(value=40.0)
        self.dy = tk.DoubleVar(value=0.0)
        self.dz = tk.DoubleVar(value=0.0)
        self.drx = tk.DoubleVar(value=0.0)
        self.dry = tk.DoubleVar(value=0.0)
        self.drz = tk.DoubleVar(value=0.0)
        self._row_inputs(self.grp_trel, ['ΔX','ΔY','ΔZ','ΔRx','ΔRy','ΔRz'],
                         [self.dx,self.dy,self.dz,self.drx,self.dry,self.drz])

        # 実行ボタン
        runf = ttk.Frame(self.root); runf.pack(fill='x', **pad)
        ttk.Button(runf, text='Move 実行', command=self.on_move).pack(side='left', padx=4)

        # 初期表示調整
        self._on_mode_change()

    # ---------- UI helpers ----------
    def _row_inputs(self, parent, labels, variables):
        for i, (lbl, var) in enumerate(zip(labels, variables)):
            ttk.Label(parent, text=lbl).grid(row=0, column=i*2, sticky='e')
            ttk.Entry(parent, width=10, textvariable=var).grid(row=0, column=i*2+1, sticky='w')

    def _set_group_state(self, frame: ttk.LabelFrame, enabled: bool):
        state = 'normal' if enabled else 'disabled'
        for w in frame.winfo_children():
            try:
                w.configure(state=state)
            except tk.TclError:
                pass

    def _on_mode_change(self):
        m = self.mode.get()
        self._set_group_state(self.grp_abs,  enabled=(m in ('MovJ','MovL')))
        self._set_group_state(self.grp_jrel, enabled=(m == 'RelMovJ'))
        self._set_group_state(self.grp_trel, enabled=(m == 'RelMovL'))

    # ---------- Callbacks ----------
    def on_enable(self):
        try:
            # demo.py と同じ初期化（Enable + SpeedFactor=10）
            self.node.initialization()
        except Exception as e:
            messagebox.showerror('Error', f'Enable/初期化に失敗: {e}')

    def on_disable(self):
        try:
            self.node.finish()
        except Exception as e:
            messagebox.showerror('Error', f'Disableに失敗: {e}')

    def on_get_pose(self):
        try:
            self.node.GetPose()
            pose = getattr(self.node, 'pose', None)
            if isinstance(pose, list) and len(pose) >= 6:
                self.ax.set(pose[0]); self.ay.set(pose[1]); self.az.set(pose[2])
                self.arx.set(pose[3]); self.ary.set(pose[4]); self.arz.set(pose[5])
            else:
                messagebox.showwarning('Warn', 'Poseの取得に失敗（値が不正）')
        except Exception as e:
            messagebox.showerror('Error', f'GetPoseに失敗: {e}')

    def on_apply_speed(self):
        try:
            sf = max(1, min(100, int(self.v_sf.get())))
            al = max(1, min(100, int(self.v_al.get())))
            sl = max(1, min(100, int(self.v_sl.get())))
            cp = max(1, min(100, int(self.v_cp.get())))
            self.node.set_parameter_L(sf, al, sl, cp)
        except Exception as e:
            messagebox.showerror('Error', f'速度パラメータ設定に失敗: {e}')

    def on_move(self):
        """demo.py の point() をそのまま使って各モードを実行"""
        m = self.mode.get()
        try:
            if m == 'MovJ':
                self.node.point('MovJ', self.ax.get(), self.ay.get(), self.az.get(),
                                self.arx.get(), self.ary.get(), self.arz.get())
            elif m == 'MovL':
                self.node.point('MovL', self.ax.get(), self.ay.get(), self.az.get(),
                                self.arx.get(), self.ary.get(), self.arz.get())
            elif m == 'RelMovJ':
                self.node.point('RelMovJ', self.dj1.get(), self.dj2.get(), self.dj3.get(),
                                self.dj4.get(), self.dj5.get(), self.dj6.get())
            elif m == 'RelMovL':
                self.node.point('RelMovL', self.dx.get(), self.dy.get(), self.dz.get(),
                                self.drx.get(), self.dry.get(), self.drz.get())
            else:
                messagebox.showerror('Error', f'未対応のモード: {m}')
        except Exception as e:
            messagebox.showerror('Error', f'Move実行に失敗: {e}')

    def on_home(self):
        """ホームポジションへ MovJ で移動"""
        if messagebox.askokcancel('Home', 'ホームポジションへMovJで移動しますか？\n周囲の安全を確認してください。'):
            try:
                self.node.point('MovJ', *HOME_POSE)
            except Exception as e:
                messagebox.showerror('Error', f'Home移動に失敗: {e}')

    def on_exit(self):
        """Disableして安全に終了（UIブロック回避のためスレッドで実行）"""
        if messagebox.askokcancel('終了', 'ロボットをDisableしてアプリを終了しますか？'):
            def _work():
                try:
                    self.node.finish()  # Disable など
                except Exception:
                    pass
                finally:
                    self.root.after(0, self.root.destroy)
            threading.Thread(target=_work, daemon=True).start()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = adderClient("nova5_gui_client")   # demo.py の adderClient を再利用
    try:
        app = NovaGUI(node)
        app.run()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
