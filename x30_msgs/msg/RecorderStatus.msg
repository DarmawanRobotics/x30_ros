# recorder_node -> recorder/status (latched, 1 Hz): what the perception host records, where.

std_msgs/Header header
bool recording
string reason             # patrol | manual (empty: not recording)
string session            # the session's directory
float32 duration_sec
uint32 thermal_frames     # raw thermal frames written (.npz)
uint32 acoustic_chunks    # acoustic data files written (.npz)
uint32 video_files        # video segments started (.mp4)
uint32 dropped            # frames not written in time (disk too slow)
string[] streams          # name: recording | restarting | off
float32 free_gb           # on the recordings' disk
string message            # what is wrong, if anything
