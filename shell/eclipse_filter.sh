# !/bin/bash

#DIR_OUT_FILE=.filterdir
DIR_OUT_OFILE=.odir
FILTER_VALUE=
FILTER_FILE="*.o *.ko *.dtb"
WORKSPACE=$2/

function file_exist()
{
	local file=
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
	local FILE_EXIT=0
	local file=

	for file in `ls "$1"`;do
		if [ -d "$1$file" ] && [ "$file" != "include" ];then
			filter_dir "$1$file/"
			if [ $? -eq 1 ];then
				FILE_EXIT=1
			fi

		fi
	done

	pro_dir=$1
	pro_dir=${pro_dir#*$WORKSPACE}
	file_exist $1

	if [ $? -eq 1 ];then
		FILE_EXIT=1
	fi

	if [ $FILE_EXIT -ne 1 ];then
		# echo "$pro_dir" >> $DIR_OUT_FILE
		FILTER_VALUE+="$pro_dir|"
	else
		echo "$pro_dir" >> $DIR_OUT_OFILE
	fi
#	echo $1 $FILE_EXIT >> .out
	return $FILE_EXIT
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

([ -z "$1" ] || [ -z "$2" ]) && echo " \$1 project path, \$2 project name" && exit

rm $DIR_OUT_OFILE .$2_cproject > /dev/null

filter_dir $1
filter_cfile $1
echo $FILTER_VALUE > .$2_out

FILTER_VALUE+="firmware/|scripts/|tools/|doc/Documentation|samples|test|Debug"

# filter unused file, can display in eclipse
[ -z $FILTER_VALUE ] && echo "filer out file no exist" && exit

if [ -f $1/.cproject ];then
	while read line
	do
		echo $line | grep "excluding=" > /dev/null
		if [ $? -ne 0 ];then
			echo $line >> .$2_cproject
		else
			cat >> .$2_cproject << EOF
						<entry excluding="$FILTER_VALUE" flags="VALUE_WORKSPACE_PATH" kind="sourcePath" name=""/>
EOF
		fi
	done < $1/.cproject
	cp .$2_cproject $1/.cproject
fi

# filter unused file, no display in eclipse
if [ -f $1/.project ];then
	sed -i 's/<arguments>1.0-name-matches-false-true-.*/<arguments>1.0-name-matches-false-true-.*\.o|.*\.ko|.*\.order|.*builtin|Makefile|.*config<\/arguments>/' $1/.project
	sed -n 's/<arguments>1.0-projectRelativePath-matches-false-true-.*/<arguments>1.0-projectRelativePath-matches-false-true-tools|scripts|samples<\/arguments>/' $1/.project
fi