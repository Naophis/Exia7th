function build(tgt_model_name)

    tgt_model_name = 'mpc_tgt_calc';

    home = pwd;
    build_home = which(tgt_model_name);
    cd(fileparts(build_home));

    tgt_dir_name = 'gen_code';

    mkdir('build');
    mkdir(tgt_dir_name);
    cd(tgt_dir_name);
    gen_code_dir = pwd;
    cd ..;
    tgtdir = strcat('../../', tgt_dir_name);

    cd build;

    slbuild(tgt_model_name);

    dir_list = dir;
    list_size = size(dir_list, 1);
    tgt_name = '';

    for i = 1:1:list_size
        dir_name = dir_list(i).name;
        res = contains(dir_name, 'ert');

        if res
            tgt_name = dir_name;
            break;
        end

    end

    cd(tgt_name);
   
    copyfile('*.c*', tgtdir);
    copyfile('*.h', tgtdir);

    cd(fileparts(build_home));
    cd ..;
    cd include;
    copyfile('*.h', gen_code_dir);
    
    cd(gen_code_dir);
%    delete ert_main.c   
    
    copyfile('*.*', '../../../src/gen_code');
    cd(home);

end
