filename=open3d-devel-linux-x86_64-cxx11-abi-cuda-0.16.0
wget -nc https://github.com/isl-org/Open3D/releases/download/v0.16.0/$filename.tar.xz
tar -xvf $filename.tar.xz 
sudo rsync -rvP $filename/* /usr/local
rm -r $filename
rm $filename.tar.xz
