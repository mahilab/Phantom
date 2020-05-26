function cpp = generate_code(x,var_str)

cpp = ccode(x);

cpp = strrep(cpp,'sin(q1)','s1');
cpp = strrep(cpp,'cos(q1)','c1');
cpp = strrep(cpp,'sin(q2)','s2');
cpp = strrep(cpp,'cos(q2)','c2');
cpp = strrep(cpp,'sin(q3)','s3');
cpp = strrep(cpp,'cos(q3)','c3');
cpp = strrep(cpp,'sin(q2-q3)','s23');
cpp = strrep(cpp,'sin(q3-q2)','s32');
cpp = strrep(cpp,'cos(q2-q3)','c23');
cpp = strrep(cpp,'cos(q3-q2)','c32');

% cpp = strrep(cpp,'[0][0]','(0,0)');
% cpp = strrep(cpp,'[0][1]','(0,1)');
% cpp = strrep(cpp,'[0][2]','(0,2)');
% cpp = strrep(cpp,'[1][0]','(1,0)');
% cpp = strrep(cpp,'[1][1]','(1,1)');
% cpp = strrep(cpp,'[1][2]','(1,2)');
% cpp = strrep(cpp,'[2][0]','(2,0)');
% cpp = strrep(cpp,'[2][1]','(2,1)');
% cpp = strrep(cpp,'[2][2]','(2,2)');

cpp = strrep(cpp,'_a','_a.');
cpp = strrep(cpp,'_c','_c.');
cpp = strrep(cpp,'_be','_be.');
cpp = strrep(cpp,'_df','_df.');
cpp = strrep(cpp,'_g','_g.');

cpp = strrep(cpp,'m_a.','m_a');
cpp = strrep(cpp,'m_c.','m_c');
cpp = strrep(cpp,'m_be.','m_be');
cpp = strrep(cpp,'m_df.','m_df');
cpp = strrep(cpp,'m_g.','m_g');

if (min(size(x)) == 1)
cpp = strrep(cpp,'x[',[var_str '[']);
cpp = strrep(cpp,'][0]',']');    
else
cpp = strrep(cpp,'x[',[var_str '(']);
cpp = strrep(cpp,'][',',');
cpp = strrep(cpp,']',')');
end

end