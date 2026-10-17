# Autonomous asynchronous rollout collection (see rollout.py).
#
# Usage:
#   bash rollout.sh --task_name <name> [more rollout.py args...]
#
# Everything you pass here is appended verbatim to rollout.py, so any argument
# rollout.py accepts works as a trailing flag (later flags override earlier ones):
#   --task_name <name>      choose the task / output directory   (required)
#   --dataset_dir <dir>     output root                          (default ./data)
#   --episode_idx <n>       start index                          (default: count existing)
#   --dry_run               run scheduling+recording without moving the robot
#   --resume                only move the arm to the reset pose, then exit
#   --record_rate fixed --record_hz <hz>   fixed-rate sampling instead of feedback
#   --trace_images          also store the 224x224 observation images
#   --frame_rate <hz>       command-publish rate (default 50; see note below)
#
# Examples:
#   bash rollout.sh --task_name pick_package
#   bash rollout.sh --task_name pick_package --dataset_dir ./data
#   bash rollout.sh --task_name pick_package --dry_run
#   bash rollout.sh --resume                       # just lower the arm
#
# During the run:
#   s = start / end recording      n = end episode and save
#   r = discard episode            t = annotate one newly confirmed delivery
# The online task script can drive the same events over ZMQ (port 5604):
#   episode_start / episode_end:<reason> / outcome:<reason>

# --task_name decides where episodes are written, so require it explicitly
# (--resume is exempt: it only moves the arm and exits without recording).
case " $* " in
    *" --task_name "*|*" --task_name="*|*" --resume "*|*" --resume="*) ;;
    *) echo "rollout.sh: pass --task_name <name>, e.g. bash rollout.sh --task_name my_task" >&2
       echo "             (or 'bash rollout.sh --resume' to just lower the arm)" >&2
       exit 2 ;;
esac

python rollout.py \
    --config_path config_multi_inference.yml \
    --contiguous \
    --server_host 127.0.0.1 \
    --model openeai \
    --pos_lookahead_step 2 \
    --pos_inference_step 24 \
    --frame_rate 50 \
    --task_instruction "Move the packages to the conveyor belt on the right" \
    --merge --stand \
    --zmq_port 5604 \
    "$@"
    # --frame_rate is the COMMAND-publish rate, not the recording rate: the recorded
    # trajectory is always on the 50 Hz arm-feedback timebase (matching the SFT data),
    # so raising it does not change the dataset -- it only changes how often a command
    # is sent (and therefore the physical duration of a model action chunk). Keep it at
    # the training timebase (50) for collection; override with --frame_rate <hz>.
