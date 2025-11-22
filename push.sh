#!/bin/bash

# 获取当前分支
BRANCH=$(git symbolic-ref --short HEAD)

# 读取提交信息，默认 "update"
read -p "请输入提交信息（默认: update）: " COMMIT_MSG
COMMIT_MSG=${COMMIT_MSG:-update}

# 添加所有修改
git add -A

# 提交
git commit -m "$COMMIT_MSG"

# 获取远程仓库 URL
REMOTE_URL=$(git remote get-url origin)

# 判断是否是 SSH 还是 HTTPS
if [[ $REMOTE_URL == https://* ]]; then
    echo "使用 HTTPS 推送，增加超时设置以防网络慢..."
    git -c http.postBuffer=524288000 push origin "$BRANCH"
elif [[ $REMOTE_URL == git@* ]]; then
    echo "使用 SSH 推送..."
    git push origin "$BRANCH"
else
    echo "无法识别远程仓库类型，请检查 git remote"
    exit 1
fi

# 检查 push 是否成功
if [ $? -eq 0 ]; then
    echo "✅ 推送成功！"
else
    echo "❌ 推送失败，请检查网络或远程仓库配置"
fi
