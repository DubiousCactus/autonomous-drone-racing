.SILENT:
## Main document
MAINFILE = main

# Build Ordner
BUILDDIR = ./ # ./build/
# Latex-Files Ordner
TEXFILES = ./ # src/



# Commands
TEX = pdflatex	
TEXFLAGS =-file-line-error -interaction=batchmode# -synctex=1 -output-directory=$(BUILDDIR)
BIB = biber
INDEX = makeindex
DEL = rm -rf 
ECHO = echo -e 
ECHOCOLOR = \033[1;32m
ECHONC = \033[0m
MOVE = mv
VIEWPDF = evince
PARSELOG = grep --color -E ".*:[0-9]*:.*|Warning:|Error:" $(BUILDDIR)$(MAINFILE).log


.PHONY: all 
all: pdf1 pdf2
	@$(MOVE) $(BUILDDIR)$(MAINFILE).pdf .

.PHONY: pdf
pdf: pdf1
	@$(MOVE) $(BUILDDIR)$(MAINFILE).pdf .

pdf1:
	mkdir -p build/
	@$(ECHO) "$(ECHOCOLOR)""\\n======================================================================="
	@$(ECHO) "Build PDF..."
	@$(ECHO) "=======================================================================""$(ECHONC)"
	$(TEX) $(TEXFLAGS) $(MAINFILE).tex 
	@$(ECHO) "$(ECHOCOLOR)""\\n======================================================================="
	@$(ECHO) "Done!!"
	@$(ECHO) "=======================================================================""$(ECHONC)"

pdf2: 
	@$(ECHO) "$(ECHOCOLOR)""\\n======================================================================="
	@$(ECHO) "Build PDF..."
	@$(ECHO) "=======================================================================""$(ECHONC)"
	$(TEX) $(TEXFLAGS) $(MAINFILE).tex 
	@$(ECHO) "$(ECHOCOLOR)""\\n======================================================================="
	@$(ECHO) "Done!!"
	@$(ECHO) "=======================================================================""$(ECHONC)"
	
pdf3:
	
	@$(ECHO) "$(ECHOCOLOR)""\\n======================================================================="
	@$(ECHO) "Build PDF again..."
	@$(ECHO) "=======================================================================""$(ECHONC)"
	$(TEX) $(TEXFLAGS) $(MAINFILE).tex 
	@$(ECHO) "$(ECHOCOLOR)""\\n======================================================================="
	@$(ECHO) "Done!!"
	@$(ECHO) "=======================================================================""$(ECHONC)"
	
.PHONY: bib
bib:
	
	@$(ECHO) "$(ECHOCOLOR)""\\n======================================================================="
	@$(ECHO) "Build bibliography..."
	@$(ECHO) "=======================================================================""$(ECHONC)"
	$(BIB) $(BIBFLAGS) $(BUILDDIR)$(MAINFILE)
	@$(ECHO) "$(ECHOCOLOR)""\\n======================================================================="
	@$(ECHO) "Done!!"
	@$(ECHO) "=======================================================================""$(ECHONC)"

.PHONY: glos
glos:
	
	@$(ECHO) "$(ECHOCOLOR)""\\n======================================================================="
	@$(ECHO) "Build glossary..."
	@$(ECHO) "=======================================================================""$(ECHONC)"
	$(INDEX) -s $(BUILDDIR)$(MAINFILE).ist -t $(BUILDDIR)$(MAINFILE).glg -o $(BUILDDIR)$(MAINFILE).gls $(BUILDDIR)$(MAINFILE).glo
	@$(ECHO) "$(ECHOCOLOR)""\\n======================================================================="
	@$(ECHO) "Build list of abbreviations..."
	@$(ECHO) "=======================================================================""$(ECHONC)"
	$(INDEX) -s $(BUILDDIR)$(MAINFILE).ist -t $(BUILDDIR)$(MAINFILE).alg -o $(BUILDDIR)$(MAINFILE).acr $(BUILDDIR)$(MAINFILE).acn 
	@$(ECHO) "$(ECHOCOLOR)""\\n======================================================================="
	@$(ECHO) "Done!!"
	@$(ECHO) "=======================================================================""$(ECHONC)"
	
	
# Clean Working Directory
clean:
	
	@$(ECHO) "$(ECHOCOLOR)""\\n======================================================================="
	@$(ECHO) "Clean Working Directory...""$(ECHONC)"
	$(DEL) $(BUILDDIR)*
	$(DEL) $(TEXFILES)*.bak
	$(DEL) $(TEXFILES)*.~	
	@$(ECHO) "$(ECHOCOLOR)""Done!"
	@$(ECHO) "=======================================================================""$(ECHONC)"

mrproper:
	
	@$(ECHO) "$(ECHOCOLOR)""\\n======================================================================="
	@$(ECHO) "Clean Working Directory...""$(ECHONC)"
	$(DEL) $(BUILDDIR)*
	$(DEL) $(TEXFILES)*.bak
	$(DEL) $(TEXFILES)*.~	
	$(DEL) $(MAINFILE).pdf
	@$(ECHO) "$(ECHOCOLOR)""Done!"
	@$(ECHO) "=======================================================================""$(ECHONC)"
	
# View PDF document
view: viewpdf

viewpdf:
	
	@$(ECHO) "$(ECHOCOLOR)""======================================================================="
	@$(ECHO) "View PDF..."
	@$(ECHO) "=======================================================================""$(ECHONC)"
	if [ -a  $(BUILDDIR)$(MAINFILE).pdf ]; then $(MOVE) $(BUILDDIR)$(MAINFILE).pdf . ; fi;
	@$(VIEWPDF) $(MAINFILE).pdf &
	
	

# View Warning/Error Messages
viewlog:
	
	@$(ECHO) "$(ECHOCOLOR)""======================================================================="
	@$(ECHO) "Warnings:\\n"
	@$(PARSELOG)
	@$(ECHO) "\\n=======================================================================""$(ECHONC)"


 
