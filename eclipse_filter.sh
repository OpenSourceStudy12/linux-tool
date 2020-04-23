# !/bin/bash

DIR_OUT_FILE=.filterdir
DIR_OUT_OFILE=.odir
FILTER_VALUE=
FILTER_FILE="*.c *.o *.dtb"
WORKSPACE=$2/

function file_exist()
{
	for file in $FILTER_FILE
	do
		res=`find $1 -maxdepth 1 -name $file`
		if [ -n "$res" ];then
			return 1
		fi
	done
	return 0
}

function filter_dir()
{
	DIR_EXIT=
	for file in `ls "$1"`
	do
		if [ -d "$1$file" ] && [ "$file" != "include" ];then
			filter_dir "$1$file/"
			DIR_EXIT=1
		fi
	done
	pro_dir=$1
	pro_dir=${pro_dir#*$WORKSPACE}
	file_exist $1
	if [ $? -ne 1 ];then
		if [ -z "$DIR_EXIT" ];then
			echo "$pro_dir" >> $DIR_OUT_FILE
			FILTER_VALUE+="$pro_dir|"
		fi
	else
		echo "$pro_dir" >> $DIR_OUT_OFILE
	fi
}

function filter_cfile()
{
	while read dir
	do
		for cfile in `find "$1$dir" -maxdepth 1 -name "*.c"`
		do
			o_file=${cfile%c*}o
			if [ ! -f "$o_file" ];then
				pro_cfile=$cfile
				pro_cfile=${pro_cfile#*$WORKSPACE}
				FILTER_VALUE+="$pro_cfile|"
			fi
		done
	done < $DIR_OUT_OFILE
}

[ -f "$DIR_OUT_FILE" ] && rm $DIR_OUT_FILE; rm $DIR_OUT_OFILE; rm .out 
[ -z "$1" ] && exit

filter_dir $1
filter_cfile $1
echo $FILTER_VALUE > .$2_out
#rm $DIR_OUT_FILE; rm $DIR_OUT_OFILE;