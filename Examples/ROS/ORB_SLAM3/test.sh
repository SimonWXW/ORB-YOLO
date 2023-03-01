#!/bin/bash
gnome-terminal --tab --title="roscore" --command="bash -c 'gcc -v; exec bash'"
gcc -v
g++ -v

#说明：nohup加在一个命令的最前面，表示不挂断的运行命令
#-u 表示实时输出到.out
#&加在一个命令的最后面，表示这个命令放在后台执行
