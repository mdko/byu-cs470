g++ -std=c++11 -g -o ./kalman -O2 -L ./lib/usr/lib64/ -larmadillo ./cpp_code/kalman.cpp ./cpp_code/Image.cpp -I ./cpp_code/ -I ./ -I ./lib/usr/include/
g++ -std=c++11 -g -o ./skeet -O2 ./cpp_code/skeet.cpp ./cpp_code/Image.cpp -I ./cpp_code/ -I ./
echo "ALERT: You need to use this command before you run this app:"
echo "\"export LD_LIBRARY_PATH=/users/guest/m/mikeac89/cs470/bzrflag/lib/usr/lib64\""
