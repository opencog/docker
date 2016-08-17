# For more see http://wiki.opencog.org/w/Development_standards#GDB

# ----------------------------------------------------------------------------
# Guile garbage collection uses these, ignore them.
# ----------------------------------------------------------------------------
handle SIGPWR noprint nostop
handle SIGXCPU noprint nostop

# ----------------------------------------------------------------------------
# Print OpenCog Objects
# ----------------------------------------------------------------------------
set overload-resolution on

define poc
	if $argc == 0
		help patom
	else
		whatis $arg0
		printf "%s", opencog::oc_to_string($arg0)._M_dataplus._M_p
	end
end

document poc
	Prints opencog object information.
	Syntax: poc <opencog object>
	Example:
	poc h - Prints the hypergraph corresponding to h
end

# ----------------------------------------------------------------------------
# Print opencog::AtomSpace
# ----------------------------------------------------------------------------
define patomspace
	if $argc == 0
		help patomspace
	else
		printf "%s", $arg0.to_string()._M_dataplus._M_p
	end
end

document patomspace
	Prints opencog::AtomSpace information.
	Syntax: patomspace <atomspace>
	Example:
	patomspace as - Prints the whole atomspace as
end
