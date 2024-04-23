cd basketball
for num in {0..29}
do 
   ffmpeg.exe -y -i $num.png -pix_fmt rgb24 $num.yuv
done


