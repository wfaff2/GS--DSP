source devel/setup.bash
roslaunch coni_mpc num_sim_non_one_point.launch r:=1.0 v:=0.5 w:=0.25



catkin_make -DCMAKE_BUILD_TYPE=Release


source devel/setup.bash
python3 scripts/run_metrics_suite.py --out results/metrics_summary.csv

cd /home/wfaff2/桌面/CoNi-MPC\(复件\)
./scripts/launch_compare_rviz.sh


消融CBF实验
KP=1.0 METRIC_SCOPE=all ./scripts/launch_astar_compare_kp_rviz.sh
非惯性VS惯性
PLOT_UAV_IDX=all SEED_START=1 SEED_END=9 USE_RVIZ=true ./scripts/launch_compare_rviz.sh

frozen VS stage 
PLOT_UAV_IDX=all SEED_START=1 SEED_END=9 USE_RVIZ=false ./scripts/launch_compare_rviz.sh



