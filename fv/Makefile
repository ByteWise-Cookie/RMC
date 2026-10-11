# Formal verification runs (SymbiYosys).
#   make            run every task of every module
#   make rob        run every task of rmc_cif_rob
#   make rob T=d4_prove   run one task
#   make traces M=rob T=d4_cover          list which trace shows which property
#   make wave   M=rob T=d4_cover N=4      open trace4.vcd in gtkwave
#   make clean
#
# Tools come from OSS CAD Suite; override OSS_CAD if it lives elsewhere.

OSS_CAD ?= $(HOME)/.local/opt/oss-cad-suite
export PATH := $(OSS_CAD)/bin:$(PATH)

WORK := work

# short name -> .sby file
SBY_rob := cif/rmc_cif_rob/rmc_cif_rob.sby

MODULES := rob

.PHONY: all $(MODULES) traces wave clean
all: $(MODULES)

$(MODULES):
	@mkdir -p $(WORK)
	sby -f --prefix $(WORK)/$@ $(SBY_$@) $(T)

# cover traces are trace<N>.vcd; a failed proof leaves trace.vcd (bmc, from
# reset) or trace_induct.vcd (induction step, starts from an arbitrary state)
traces:
	@grep -hE "Reached cover|Assert failed|Writing trace to VCD" $(WORK)/$(M)_$(T)/logfile.txt | sed 's/.*##  *[0-9:]*  //'

wave:
	gtkwave $(WORK)/$(M)_$(T)/engine_0/trace$(N).vcd &

clean:
	rm -rf $(WORK)
