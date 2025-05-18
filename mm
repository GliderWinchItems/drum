#!/bin/bash -e
# Example: Specify CAN ID to select parameter files
# e.g. CAN ID = 80000000
# ./mm 80000000 

#rm build/shaft_drum.elf
export FLOAT_TYPE=hard

# Parameters for CAN ID specified by $1

# ---- adc parameters
if [ -r params/$1-adc_idx_v_struct.c ]; then
	export ADC_PARAM=$1-adc_idx_v_struct.c
else
	echo params/$1-adc_idx_v_struct.c does not exist
	exit 1;
fi	

# ---- odometer function parameters (en == encoder)
if [ -r params/$1-en_idx_v_struct.c ]; then
	export EN_PARAM=$1-en_idx_v_struct.c
else
	echo params/$1-en_idx_v_struct.c does not exist
	exit 3;
fi	

# ---- levelwind function parameters
if [ -r params/$1-levelwind_idx_v_struct.c ]; then
	export LVL_PARAM=$1-levelwind_idx_v_struct.c
else
	echo params/$1-levelwind_idx_v_struct.c does not exist
	exit 3;
fi	

echo "################ CAN ID  ################"
echo $1
echo "################ PARAMETER FILES ################"
echo $ADC_PARAM
echo $EN_PARAM
echo $LVL_PARAM

export I_AM_CANID=0x$1
echo I_AM_CANID
echo $I_AM_CANID
make clean
./script-all drum $1

exit

# ----saved stuff----
#!/bin/bash
export FLOAT_TYPE=hard

export I_AM_CANID=0x$1
echo I_AM_CANID
make clean

rm build/drum.elf
make

./script-all drum

