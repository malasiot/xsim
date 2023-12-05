mkdir cropped
for f in *.png; do
    convert "$f" -crop 390x390+300+500 cropped/"$f" 
done
