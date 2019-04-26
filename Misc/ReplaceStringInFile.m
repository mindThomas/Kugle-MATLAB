function ReplaceStringInFile(filename, strToFind, strToReplaceWith)
    tmpfilename = ['tmp_' filename];
    fidi=fopen(filename,'r');
    fido=fopen(tmpfilename,'w');
    while ~feof(fidi)
      l=fgetl(fidi);   % read line
      l = strrep(l, strToFind, strToReplaceWith);
      fprintf(fido,'%s\n',l);  % 'fgetl returns \n so it's embedded
    end
    fidi=fclose(fidi);
    fido=fclose(fido);
    delete(filename);
    movefile(tmpfilename, filename);
end