#./bin/bzrflag --world=maps/four_ls.bzw --friendly-fire --red-port=50100 --green-port=50101 --purple-port=50102 --blue-port=50103 $@ &
#./bin/bzrflag --world=maps/four_ls.bzw --friendly-fire --red-port=50100 --green-port=50101 --default-tanks=2 $@ &
#sleep 2

#./start_server.sh
#sleep 2
#./search localhost 50100 dfs 2
python ./bzagents/agent0.py localhost 50101 &
./yoda localhost 50100 800 6 1 50 1
#                        World Grid Size (400))
#                            ShotSpeed (Higher is closer to target/behind if too big, lower leads ahead of the target) (8)
#                              Bullets on/off (1)
#                                Position Confidence (50)
#                                    Acceleration Confidence (1)
						  
#./kalman localhost 50100 400 8 1 50 1
#./search localhost 50100 bfs
#./search localhost 50100 iddfs
#./search localhost 50100 ucost 1 1
#./search localhost 50100 astar 1 1

#./bzagents_cpp/potential_field_agent localhost 50100 red &
#./bzagents_cpp/potential_field_agent localhost 50102 green &
#./bzagents_cpp/dummy_agent localhost 50101 &
#./bzagents_cpp/dummy_agent localhost 50103 &
