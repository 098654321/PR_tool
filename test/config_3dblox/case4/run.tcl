set pkg [file dirname [info script]]
set out [file join $pkg results]
read_3dbx [file join $pkg case4.3dbx]
prtool_verify_import [file join $pkg prtool_reference] -report [file join $out prtool_import_check.log]
prtool_route -router maze -verbose -output_dir $out
write_db [file join $out case4_prtool.odb]
