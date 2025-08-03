#!/bin/bash

# 移動先ディレクトリ
cd ~/dobot_ws || exit

# git add
git add .

# git status
git status

# 現在の日時を取得
current_date=$(date "+%Y-%m-%d %H:%M:%S")

# git commit
git commit -m "git push $current_date"

# git push
git push origin main

