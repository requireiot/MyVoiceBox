
# calculate lookup table to transform psychophysical brightness to LED PWM
# inspired by
#  https://ledshield.wordpress.com/2012/11/13/led-brightness-to-your-eye-gamma-correction-no/
#  https://gist.github.com/mathiasvr/19ce1d7b6caeab230934080ae1f1380e

INPUT_SIZE = 255       # Input integer size
OUTPUT_SIZE = 4095      # Output integer size
INT_TYPE = 'const uint16_t'
TABLE_NAME = 'cie';

def cie1931(L):
    L = L*100.0
    if L <= 8:
        return (L/902.3)
    else:
        return ((L+16.0)/116.0)**3

x = range(0,int(INPUT_SIZE+1))
y = [round(cie1931(float(L)/INPUT_SIZE)*OUTPUT_SIZE) for L in x]

f = open('../main/cie1931.h', 'w')
f.write('// CIE1931 correction table\n')
f.write('// Automatically generated\n\n')

f.write('%s %s[%d] = {\n' % (INT_TYPE, TABLE_NAME, INPUT_SIZE+1))
f.write('\t')
for i,L in enumerate(y):
    f.write('%d, ' % int(L))
    if i % 10 == 9:
        f.write('\n\t')
f.write('\n};\n\n')