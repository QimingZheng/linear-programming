## Generate testing files in the `datasets/` folder
gcc emps.c -o emps
for file in ../datasets/netlib/*;do
    if [[ $file != *.txt ]] && [[ $file != *.mps ]];
    then
        ./emps $file > ${file}.mps
        python3 mps_load.py ${file}.mps ${file}.txt
    fi
done

for file in ../datasets/plato/*;do
    if [[ $file == *.mps ]];
    then
        python3 mps_load.py ${file} ${file}.txt
    fi
done
