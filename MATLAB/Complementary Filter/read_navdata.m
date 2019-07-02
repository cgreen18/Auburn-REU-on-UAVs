function navdata = read_navdata(file_name)
    
    file_table = readtable(file_name);
    
    file_data = table2array(file_table);
    
    navdata = file_data;
    
end
