### 安装eigen_macOS 本地安装 Eigen

1. 使用brew安装
```bash
brew install eigen
```
安装完成后，eigen文件夹位于 /usr/local/Cellar/eigen/3.3.7/include/eigen3 文件夹内，并非系统文件夹，因此还需下面一步

2. 链接到系统文件夹
```bash
brew link --overwrite eigen
```
这样就会从brew的安装目录，产生一个链接到/usr/local/bin，文件夹下创建一个文件夹链接(名为eigen3)指向上面的真实eigen文件夹