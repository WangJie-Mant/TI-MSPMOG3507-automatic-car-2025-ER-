### 开发环境准备

1. 前往德州半导体官网，搜索MSPM0G3507,点击"评估板"下面的链接
2. 下载两个东西：**SDK**和**CCSTUDIO**，一个是开发工具链，一个是IDE.下载并安装。
3. （可选）同样的页面里可以下载一个uniflash用来烧录
4. 安装CCSTUDIO的过程中会出现选择安装，按照需要选择即可，我只选择了MSPM0G3507
5. 打开CCSTUDIO，ctrl+shift+p搜索language，出现**configure display language**设置项，等它转一会，然后下载中文语言包，下载完成后重启应用

### 新建工程

1. 文件-->new project打开project wizard

2. 搜索MSPM0G3507，选择**device**为该型号的开发板，下面两个复选框选择**nortos** **CCS - TI Arm Clang Compiler**，password和category留空就行

3. create

4. 右键工程，选择**properties**进入首选项，找到**build**，完成以下选项：

   **以下步骤是配合uniflash烧录工具使用的，可以选择忽略**

   1. arm Hex Utility	左下角勾选**Enable**
   2. 展开的选项卡中选择**general options**，将**Specify memory width (--memwidth, -memwidth)**和**Specify rom width (--romwidth, -romwidth)**填为8
   3. 展开的选项卡中选择**output format options**，选择**intel hex**

5. 配置sysconfig，编写主函数，编译，并烧录。