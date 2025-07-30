#!/bin/bash
echo "全ての起動中のDockerコンテナを停止します..."
docker ps -q | xargs -r docker stop

echo "全てのDockerコンテナの停止が完了しました。"
