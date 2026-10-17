python inference.py \
    --config_path config_multi_inference.yml \
    --contiguous \
    --server_host 127.0.0.1 \
    --model openeai \
    --pos_lookahead_step 2 \
    --pos_inference_step 24 \
    --frame_rate 50 \
    --task_instruction "Move the packages to the conveyor belt on the right" \
    --merge --stand $1
    # --smooth
