for i in **.h264; do ffmpeg -framerate 120 -i "$i" -c copy "$(basename ${i%.*}).mp4"; done
for i in **.h264; do rm "$i"; done
