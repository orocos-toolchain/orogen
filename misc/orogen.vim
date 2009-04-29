" Vim plugin to have commands useful for orogen
" 
" Installation
" ------------
" Copy orogen.vim in $HOME/.vim/plugin
"
" Documentation
" -------------
" The following commands are defined:
"  OroDiff: will diffsplit the current buffer with its template (if it exists)


if exists("loaded_orogen")
  finish
endif
let loaded_orogen = 1

" Diffs the current file with its orogen-generated template
function OroDiff(base_file)
    " Find the orogen directory by looking at *.orogen files
    let base_file = fnamemodify(a:base_file, ":p")
    let base_dir  = fnamemodify(base_file, ":h")
    let orogen_dir = finddir("templates", base_dir . ";")
    let orogen_dir = fnamemodify(orogen_dir, ":p:h:h")

    " If it is not empty, diffsplit on the template
    if orogen_dir != ""
        let template = orogen_dir . "/templates/" . fnamemodify(base_file, ":s?" . orogen_dir . "??")
        if filereadable(template)
            exec "diffsplit " template
        else
            echoerr expand('%') . " has no template file"
        endif
    endif
endfunction
command OroDiff call OroDiff(expand('%'))

