#
# guile garbage collection uses these. Ignore them.
#
handle SIGPWR noprint nostop
handle SIGXCPU noprint nostop

# ----------------------------------------------------------------------------
#
# Print opencog::Handle
#

define patom
	if $argc == 0
		help patom
	else
		printf "%s", $arg0->toString()._M_dataplus._M_p
	end
end

document patom
	Prints opencog::Handle information.
	Syntax: patom <handle>
	Example:
	patom h - Prints the hypergraph corresponding to h
end
