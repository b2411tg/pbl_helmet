#!/bin/bash

# エラー時に終了
set -e

# 仮想環境を有効化
source ~/Project/.venv/bin/activate

# プロジェクトディレクトリへ移動
cd ~/Project/pbl_helmet

# メインスクリプト実行
exec python3 helmet.py

