plot ${file_name} w l
set title "this is a test of sin"
set grid xtics ytics ls 100
set xlabel "x"
set ylabel "y"
replot

plot for [i = 0:2] "".i."data.dat"
