# Projeto Trainee UnBall 2024 Simulado

Serão alterados somente os códigos de estratégia

O código da comunicação com o Firasim e Referee podem ser encontrados em src/client

O loop principal pode ser encontrado em src/loop.py

O código que atualiza a visão pode ser encontrado em src/vision

## Histórico de competições onde foi utilizado

- IRONCUP 2024

## Para executar

Primeiro, é necessário instalar o docker na sua máquina. Após isso, crie uma conta no docker e rode o executável:
```
docker pull unball/unbrain:latest
```


Em seguida, apenas rode o executável:
```
chmod +x containerball.sh && ./containerball.sh
```

Dentro do contâiner, precisamos compilar os arquivos proto para comunicar com o
VSSSReferee (caso necessário):

```bash
cd src/client/protobuf
./protobuf.sh
```

Por fim, podemos executar o sistema como segue:

```bash
python3.8 src/main.py --team-color yellow --firasim --n_robots 0,1,2,3,4
```

### Sobre os argumentos no terminal

Podemos usar `python3.8 src/main.py --help` para saber mais sobre os argumentos que podemos
passar pelo terminal, mas os principais são

- cor do time `--team-color` que pode ser yellow ou blue
- espelhar o lado `--mirror` que contém os seguintes casos:
    - cor do time = blue e mirror desativado -> blue joga na esquerda
    - cor do time = blue e mirror ativado -> blue joga na direita
    - cor do time = yellow e mirror desativado -> yellow joga na direita
    - cor do time = yellow e mirror ativado -> yellow joga na esquerda

`--team-color` é um argumento obrigatório quando for executar no sistema, enquanto `--mirror` deve ser escrito apenas quando deseja-se ativá-lo.

Caso precise executar sem o referee, basta colar no terminal o seguinte comando
(Referee não está sendo utilizado, então o argumento está com default=True)

```bash
python3.8 src/main.py --team-color blue --immediate-start
```