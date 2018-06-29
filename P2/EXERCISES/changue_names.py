import os

os.chdir("C:/Users/juane/Dropbox/3AGI/2CUATRI/TSI/0.PRACTICAS/P2/EJERCICIOS")

archivos = os.listdir(os.getcwd())

for file in archivos:
    guion=file.find("-")

    if(guion!=-1):
        first = file[(guion+1):(guion+4)]
        second = file[0:guion]
        fin=".pddl"
        name = str(first) + str(second) + fin

        os.renames(file, name)

    # print(archivos)
