# For more see http://wiki.opencog.org/w/Development_standards#GDB

# ----------------------------------------------------------------------------
# Guile garbage collection uses these, ignore them.
# ----------------------------------------------------------------------------
handle SIGPWR noprint nostop
handle SIGXCPU noprint nostop
set print thread-events off

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

# ----------------------------------------------------------------------------
# For debugging python
# See https://wiki.python.org/moin/DebuggingWithGdb
# As of December 2021, this still runs on python2.7 and not python3
# ----------------------------------------------------------------------------
add-auto-load-safe-path  /usr/lib/debug/usr/bin/python2.7-gdb.py

# print python frame
define pyf
  if $rip >= &PyEval_EvalFrameEx
   if $rip < &PyEval_EvalCodeEx
    x/s ((PyStringObject*)f->f_code->co_filename)->ob_sval
    x/s ((PyStringObject*)f->f_code->co_name)->ob_sval
    echo py line #
    p f->f_lineno
   end
 end
end
document pyf
show python stack frame
end

# print python backtrace
define pbt
 set $i = 0
 set $j = 0
 set $prev = 0
 while $i < 105
  select $i
  if $rip >= &PyEval_EvalFrameEx
   if $rip < &PyEval_EvalCodeEx
    if $prev !=f
      echo c frame #
      p $i
      echo py frame #
      p $j
      set $j = $j+1
      x/s ((PyStringObject*)f->f_code->co_filename)->ob_sval
      x/s ((PyStringObject*)f->f_code->co_name)->ob_sval
      echo line #
      p f->f_lineno
      $prev = f
    end
   end
  end
  set $i = $i+1
 end
end
document pbt
show python backtrace
end
