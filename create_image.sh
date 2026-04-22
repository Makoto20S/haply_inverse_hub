#!/bin/bash
set -e
xhost +local:
docker compose up -d --build
echo "✅ 已重新构建，容器已在后台启动！"
echo "👉 进入容器请执行：docker exec -it haply_inverse bash"
echo "👉 使用Vscode开发请安装 Dev Containers 插件，在Vscode界面按下 F1，然后选择 'Dev Containers:Reopen in Container'"
