all:	
	@make -C lcmtypes
	@make -C dynamixel
	@make -C java


clean:
	@make -C lcmtypes -s clean
	@make -C dynamixel -s clean
	@make -C java -s clean
	@rm -f *.pyc *~
