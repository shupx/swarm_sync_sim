# git仓库使用教程（极简版）

1. 登陆gitee网站企业账号：https://e.gitee.com/bhswift/code/repos 。或者个人账号。点击右上角新建仓库。

2. 选择“普通新建”或者选择“使用模板仓库”(选择本仓库)。“路径”中输入自定义的仓库名称（英文+下划线，要简单）。

3. windows电脑百度搜索git,安装git bash。ubuntu电脑直接用如下命令安装：

```bash
sudo apt install git
```

4. 建好后点击仓库右上方按钮“克隆/下载”，复制https链接。在本地文件夹下进行仓库克隆：

```bash
git clone https://gitee.com/bhswift/4573-template-repo.git # 替换为自己的仓库地址
```

5. 将自己的代码复制到本地该仓库文件夹下，并删除模板仓库中不需要的文件，然后：

```bash
cd 4573-template-repo/ # 替换为自己的仓库名称
git add .  #添加所有新增文件
git commit -m "first commit" #输入提交消息
git push #推送到远端仓库
```
如果提示需要git config用户名和邮箱，按照提示做一次就行。之后再commit和push。

6. 之后修改代码后，需要同步到远端时：

```bash
cd 4573-template-repo/ # 替换为自己的仓库名称
git add .  #添加所有新增文件
git commit -m "commit message" #输入提交消息
git push #推送到远端仓库
```

7. 同步远端仓库的代码：

```bash
git pull
```