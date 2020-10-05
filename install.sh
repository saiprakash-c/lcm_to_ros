DIR=$PWD
cd $DIR/thirdparty

# Install cereal for message passing serialization
wget -nc https://github.com/USCiLab/cereal/archive/v1.2.1.tar.gz -O cereal.tar.gz
tar -xzf cereal.tar.gz
rsync -a cereal-1.2.1/ cereal
rm -rf cereal-1.2.1/