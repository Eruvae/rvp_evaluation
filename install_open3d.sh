filename=open3d-devel-linux-x86_64-cxx11-abi-cuda-0.15.1
wget -nc https://github.com/isl-org/Open3D/releases/download/v0.15.1/$filename.tar.xz
tar -xvf $filename.tar.xz 
sudo rsync -rvP $filename/* /usr/local
rm -r $filename
rm $filename.tar.xz
