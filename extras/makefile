all:doc
doc:
	doxygen ../extras/Doxyfile
	.\..\doc\latex\make.bat
	cp -f ../doc/latex/refman.pdf ./
	rm -rf ../doc