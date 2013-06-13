#./bin/bzrflag --world-size=800 --world=maps/pacman.bzw --occgrid-width=100 --default-true-positive=.97 --default-true-negative=.9 --no-report-obstacles --red-port=50100 --green-port=50102 --seed=1 --red-tanks=2 --green-tanks=0 --purple-tanks=0 --blue-tanks=0 $@ &
#sleep 2

#./bin/bzrflag --world-size=400 --world=maps/empty.bzw --default-posnoise=5 --red-port=50100 --green-port=50101 --seed=1 --red-tanks=1 --green-tanks=1 $@ &
#sleep 2

#./bin/bzrflag --world-size=400 --world=maps/four_ls.bzw --default-posnoise=3 --friendly-fire --respawn-time=240 --time-limit=240 --max-shots=3 --red-port=50100 --green-port=50101 --seed=1 --default-tanks=10 --red-tanks=10 --green-tanks=12 $@ &
#sleep 2

./bin/bzrflag --world-size=800 --occgrid-width=8000 --world=maps/four_ls.bzw --default-posnoise=3 --respawn-time=240 --time-limit=240 --max-shots=3 --red-port=50100 --green-port=50101 --seed=1 --red-tanks=5 --green-tanks=12 --blue-tanks=0 --purple-tanks=0 $@ &
sleep 2
