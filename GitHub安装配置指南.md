# GitHub 安装和配置完整指南

## 第一步：安装 Git

### 方法一：使用 winget（推荐，Windows 10/11）
```powershell
winget install --id Git.Git -e --source winget
```

### 方法二：手动下载安装
1. 访问 https://git-scm.com/download/win
2. 下载最新版本的 Git for Windows
3. 运行安装程序，使用默认设置即可

### 验证安装
安装完成后，重新打开终端，运行：
```powershell
git --version
```

## 第二步：配置 Git 用户信息

安装完成后，需要配置您的用户名和邮箱（这些信息会出现在您的提交记录中）：

```powershell
git config --global user.name "您的用户名"
git config --global user.email "您的邮箱@example.com"
```

**重要提示：**
- 用户名可以是任意名称，建议使用您的 GitHub 用户名
- 邮箱建议使用您注册 GitHub 时使用的邮箱

### 查看配置
```powershell
git config --global --list
```

## 第三步：连接 GitHub

有两种方式连接 GitHub：

### 方式一：使用 HTTPS（推荐新手）

1. **在 GitHub 上创建个人访问令牌（Personal Access Token）**
   - 访问 https://github.com/settings/tokens
   - 点击 "Generate new token" -> "Generate new token (classic)"
   - 设置名称（如：My Computer）
   - 选择过期时间
   - 勾选权限：至少需要 `repo` 权限
   - 点击 "Generate token"
   - **重要：复制生成的令牌，只显示一次！**

2. **使用令牌进行身份验证**
   - 当您第一次推送代码时，Git 会提示输入用户名和密码
   - 用户名：您的 GitHub 用户名
   - 密码：使用刚才生成的个人访问令牌（不是 GitHub 密码）

### 方式二：使用 SSH 密钥（更安全，推荐长期使用）

1. **生成 SSH 密钥**
   ```powershell
   ssh-keygen -t ed25519 -C "您的邮箱@example.com"
   ```
   - 按 Enter 使用默认路径
   - 可以设置密码（可选，建议设置）

2. **查看公钥内容**
   ```powershell
   cat ~/.ssh/id_ed25519.pub
   ```
   或者：
   ```powershell
   type C:\Users\您的用户名\.ssh\id_ed25519.pub
   ```

3. **将公钥添加到 GitHub**
   - 访问 https://github.com/settings/keys
   - 点击 "New SSH key"
   - Title：给密钥起个名字（如：My Laptop）
   - Key：粘贴刚才复制的公钥内容
   - 点击 "Add SSH key"

4. **测试 SSH 连接**
   ```powershell
   ssh -T git@github.com
   ```
   如果看到 "Hi 用户名! You've successfully authenticated..." 说明连接成功

## 第四步：基本使用

### 初始化仓库
```powershell
# 在当前目录创建新仓库
git init

# 或者克隆现有仓库
git clone https://github.com/用户名/仓库名.git
```

### 基本工作流程
```powershell
# 1. 查看状态
git status

# 2. 添加文件到暂存区
git add 文件名
# 或添加所有文件
git add .

# 3. 提交更改
git commit -m "提交说明"

# 4. 推送到 GitHub
git push origin main
# 如果是第一次推送
git push -u origin main
```

### 常用命令
```powershell
# 查看提交历史
git log

# 查看远程仓库
git remote -v

# 添加远程仓库
git remote add origin https://github.com/用户名/仓库名.git

# 拉取最新更改
git pull

# 创建新分支
git branch 分支名

# 切换分支
git checkout 分支名
```

## 常见问题

### Q: 如何撤销更改？
```powershell
# 撤销工作区的更改
git checkout -- 文件名

# 撤销暂存区的更改
git reset HEAD 文件名
```

### Q: 如何查看差异？
```powershell
git diff
```

### Q: 忘记配置用户信息怎么办？
重新运行配置命令即可覆盖之前的设置。

## 下一步

1. 在 GitHub 上创建一个新仓库
2. 使用 `git clone` 克隆到本地
3. 开始您的第一个项目！

---

**需要帮助？** 访问 https://docs.github.com/zh 查看官方文档
