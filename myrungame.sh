#./bin/bzrflag --world=maps/four_ls.bzw --friendly-fire --red-port=50100 --green-port=50101 --purple-port=50102 --blue-port=50103 $@ &
#./bin/bzrflag --world=maps/four_ls.bzw --friendly-fire --red-port=50100 --green-port=50101 --default-tanks=2 $@ &
#sleep 2

./start_server.sh
#./search localhost 50100 dfs 2
./search localhost 50100 dfs
#./search localhost 50100 bfs
#./search localhost 50100 iddfs
#./search localhost 50100 ucost 1 1
#./search localhost 50100 astar 1 1

#./bzagents_cpp/potential_field_agent localhost 50100 red &
#./bzagents_cpp/potential_field_agent localhost 50102 green &
#./bzagents_cpp/dummy_agent localhost 50101 &
#./bzagents_cpp/dummy_agent localhost 50103 &
