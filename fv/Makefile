# Formal verification runs (SymbiYosys).
#   make            run every task of every module
#   make rrob       run every task of rmc_cif_rrob (read ROB)
#   make rspltr     run every task of rmc_cif_r_burst_spltr
#   make rresp      run every task of rmc_cif_r_resp (read-response assembler)
#   make rrob T=d4_prove   run one task
#   make traces M=rrob T=d4_cover          list which trace shows which property
#   make wave   M=rrob T=d4_cover N=4      open trace4.vcd in gtkwave
#   make clean
#
# Tools come from OSS CAD Suite; override OSS_CAD if it lives elsewhere.

OSS_CAD ?= $(HOME)/.local/opt/oss-cad-suite
export PATH := $(OSS_CAD)/bin:$(PATH)

WORK := work

# short name -> .sby file
SBY_rrob := cif/rmc_cif_rrob/rmc_cif_rrob.sby
SBY_rspltr := cif/rmc_cif_r_burst_spltr/rmc_cif_r_burst_spltr.sby
SBY_rresp := cif/rmc_cif_r_resp/rmc_cif_r_resp.sby
SBY_tut := tutorial/tut_cnt.sby

MODULES := rrob rspltr rresp tut

.PHONY: all $(MODULES) traces wave clean
# tut is a teaching example with an expected failure; run it by name
all: $(filter-out tut,$(MODULES))

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
