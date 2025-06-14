echo "Configuring and building DBoW2 ..."
cd DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../g2o

echo "Configuring and building g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../

echo "building ncnn ..."

git clone https://github.com/Tencent/ncnn
cd ncnn
git submodule update --init

echo "Please continue to manually configure your ncnn ..."
