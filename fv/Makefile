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
# write path + shared
SBY_frptr := cif/rmc_cif_frptr/rmc_cif_frptr.sby
SBY_mcarb := cif/rmc_cif_mc_req_arb/rmc_cif_mc_req_arb.sby
SBY_awbuf := cif/rmc_cif_aw_reqbuf/rmc_cif_aw_reqbuf.sby
SBY_wdfifo := cif/rmc_cif_w_data_fifo/rmc_cif_w_data_fifo.sby
SBY_waccum := cif/rmc_cif_w_accum/rmc_cif_w_accum.sby
SBY_wspltr := cif/rmc_cif_w_burst_spltr/rmc_cif_w_burst_spltr.sby
SBY_wrob := cif/rmc_cif_wrob/rmc_cif_wrob.sby
SBY_wpktgen := cif/rmc_cif_w_pkt_gen/rmc_cif_w_pkt_gen.sby
SBY_wresp := cif/rmc_cif_w_resp/rmc_cif_w_resp.sby
SBY_tut := tutorial/tut_cnt.sby

MODULES := rrob rspltr rresp frptr mcarb awbuf wdfifo waccum wspltr wrob wpktgen wresp tut

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
