
if [ $# -ne 2 ];then
    echo "引数を２つ指定してください"
    exit  0
fi
filename="${1}_${2}.csv"
# node tools/param_tuner/sensor.js $1 $2
node tools/param_tuner/sensor.js $1 $2>$filename