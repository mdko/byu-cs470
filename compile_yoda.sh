g++ -std=c++11 -g -o ./yoda -O2 -L ./lib/usr/lib64/ -larmadillo ./cpp_code/yoda.cpp ./cpp_code/Image.cpp -I ./cpp_code/ -I ./ -I ./lib/usr/include/
echo "ALERT: You need to use this command before you run this app:"
echo "\"export LD_LIBRARY_PATH=/users/guest/m/mikeac89/cs470/bzrflag/lib/usr/lib64\""
