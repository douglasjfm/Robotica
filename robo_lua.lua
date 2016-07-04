-- DO NOT WRITE CODE OUTSIDE OF THE if-then-end SECTIONS BELOW!! (unless the code is a function definition)

if (sim_call_type==sim_childscriptcall_initialization) then

    -- This is executed exactly once, the first time this script is executed

    --Diretorio onde os dados do sonar serao gravados:
    path = "/Volumes/Dados/Dropbox/UFPE/Disciplinas/Graduacao/Robotica/Atividades/Atividade2/"

    --Nome do arquivo:
    --Arquivo contendo dados brutos do sonar+posiçao do robo
    nameRawData = "sonarDataRaw.csv"
    --Arquivo contendo a nuvem de pontos (x,y)
    nameXYData = "sonarDataXY.csv"    

    --Localizacao do objetivo inicial {x, y, beta}
    --Apos atingir este objetivo o robo seleciona outro aleatoriamente
    goals = {{-1.0, 1.5, 3.14/2}, {-0.25, 0.55, 0}, {1, -0.3, -3.14/2}, {1.0, -1.5, -3.14/2}, {1.0, 1.5, 3.14/2}, {0.25, -0.5, -3.14}, {-1.0, -1.5, -3.14/2}, {0.35, 0, 3.14/2}}
    g = 1
    goal = goals[g]

    -- Estados do robo: 
    -- 0 - indo para o objetivo; 
    -- 1 - correcao final da orientaçao
    -- 2 - chegou ao objetivo
    state=0

    --Controle de movimento:
    -- Parametros (k_rho>0; k_beta<0; k_alpha > k_rho):
    k_rho = 0.1
    k_alpha = 2
    k_beta = -0.5
    r = 0.02
    l = 0.1

    -- O seguinte código pode ser usado para criar um console extra, onde você pode imprimir
    console = simAuxiliaryConsoleOpen("Aux Console", 500, 0x10)
    simAuxiliaryConsoleShow(console,1)

    -- A linha a seguir pega a referência do objeto do qual este código faz parte (self)
    -- No caso, a referência pro "RobotFrame"
    -- Em geral, a função "simGetObjectHandle" recebe como argumento o nome do objeto que desejamos manipular
    ddRobot=simGetObjectHandle("RobotFrame")

    --Motores
    -- Os handles dos motores serão utilizados posteriormente para alterar a velocidade dos motores
    leftMotor=simGetObjectHandle("LeftMotor") -- Handle of the left motor
    rightMotor=simGetObjectHandle("RightMotor") -- Handle of the right motor

    --Sensores
    --Odometro
    graphOdometry=simGetObjectHandle("GraphOdometry")
    lwprev = simGetJointPosition(leftMotor)
    rwprev = simGetJointPosition(rightMotor)
    pos=simGetObjectPosition(ddRobot,-1)
    orientation = simGetObjectOrientation(ddRobot,-1,pos) 
    odom_x = pos[1]
    odom_y = pos[2]
    odom_theta = 2*math.pi-orientation[3] 

    --Sonares
    sonarL=simGetObjectHandle("ProximitySensorL")
    sonarF=simGetObjectHandle("ProximitySensorF")
    sonarR=simGetObjectHandle("ProximitySensorR")

    -- arquivos:
    sonarDataRawFile=io.open(path..nameRawData,"w")
    sonarDataXYFile=io.open(path..nameXYData,"w")
    
    -- Cabeçalho do arquivo do sonar
    sonarDataRawFile:write("robot_x\trobot_y\trobot_theta\talpha\tr\n")
    sonarDataXYFile:write("x\ty\n")

    ---------------------------------------------------------------
    -- Controle de velocidade pelo slider
    useSlider = false

    -- As janelas customizadas (custom UI) também são acessadas por um handle
    -- Diferentemente dos objetos da cena, a função usada é "simGetUIHandle"
    uiHandle=simGetUIHandle("SpeedControl")
    
    -- Inicialização da posição dos sliders da UI
    -- simInt simSetUISlider(simInt uiHandle,simInt buttonHandle,simInt position)
    -- Os 3 argumentos são:
    --    uiHandle - o handle da janela onde o slider se encontra
    --    buttonHandle - o id do slider em si. Cada componente da janela possui um id,
    --                   que pode ser visualizado na janela "Custom User Interfaces" do
    --                   "user interface edit mode"
    --    position - valor de 0 a 1000, que representa a posição do slider
    simSetUISlider(uiHandle,3,500) -- como 500 é o meio, o robô ficará parado
    simSetUISlider(uiHandle,1,500)
    
    -- Enable Remote API (Matlab)
    simExtRemoteApiStart(19999)

    math.randomseed(os.time())
end

---------------------------------------------------------------
-- Processamento dos Sensores (Odometros e Sonares)
---------------------------------------------------------------

-- Le dados do odometro
-- Retorna a velocidade de cada roda odom_phiL, odom_phiR
function readOdometers()
    lwcur = simGetJointPosition(leftMotor)
    rwcur = simGetJointPosition(rightMotor)

    odom_phiL=smallestAngleDiff(lwcur,lwprev)
    odom_phiR=smallestAngleDiff(rwcur,rwprev)
    lwprev = lwcur
    rwprev = rwcur

    return odom_phiL, odom_phiR
end

-- Le dados do sonar
-- Retorna -1 caso nao haja dados disponiveis neste momento
-- Retorna a distancia lida, caso contrario
function readSonar(sonar)
    local r, dist = simReadProximitySensor(sonar)

    if ((r) and (dist ~= nil)) then
        return dist
    end

    return -1
end

if (sim_call_type==sim_childscriptcall_sensing) then

    -- Processamento dos Sensores:

    --Odometro:

    odom_phiL, odom_phiR = readOdometers()

    -- Calcular a posicao do robo a partir da velocidade de cada roda
    odom_x = 0-- ???
    odom_y = 0-- ???
    odom_theta = 0-- ???

    -- Plota a linha vermelha do odometro
    simSetObjectPosition(graphOdometry, -1, {odom_x, odom_y, odom_theta})

    --Sonar:

    -- Utiliza a posiçao real do robo para os dados de sonar:
    global_x = pos[1]
    global_y = pos[2]
    global_theta = theta

    -- Utiliza a posiçao dada por odometria:
    --global_x = odom_x -- ??? Descomentar 
    --global_y = odom_y    -- ??? Descomentar
    --global_theta = odom_theta    -- ??? Descomentar

    -- Grava dados do sonar nos arquivos "sonarDataRaw.csv" e "sonarDataXY.csv"
    -- Verifica se ha dados de sonar para gravar: dist >= 0
    -- dist = distancia lida do sonar
    -- sonarF sonar da frente
    -- sonarL sonar da esquerda
    -- sonarR sonar da direita
    dist = readSonar(sonarF) -- Le o sonar da frente
    if (dist>=0) then
        --Grava dados brutos no arquivo:
        sonarDataRawFile:write(""..global_x.."\t"..global_y.."\t"..global_theta.."\t"..(0.000).."\t"..dist.."\n")

        --Calcula o ponto (x,y) em funçao de global_x, global_y, global_theta e dist:
        x = 0 -- ???
        y = 0 -- ???

        --Grava o ponto no arquivo:
        sonarDataXYFile:write(""..x.."\t"..y.."\n")
    end
   
    -- Descomentar caso queira utilizar os outros sensores sonarL e sonarR
    --[[ <--
    dist = readSonar(sonarL) -- Le o sonar da esquerda
    if (dist>=0) then
        --Grava dados brutos no arquivo:
        sonarDataRawFile:write(""..global_x.."\t"..global_y.."\t"..global_theta.."\t"..(math.pi/2).."\t"..dist.."\n")
        
        --Calcula o ponto (x,y) em funçao de global_x, global_y, global_theta e dist:
        x = 0 -- ???
        y = 0 -- ???
        
        --Grava o ponto no arquivo:
        sonarDataXYFile:write(""..x.."\t"..y.."\n")
    end
    
    dist = readSonar(sonarR) -- Le o sonar da direita
    if (dist>=0) then
        --Grava dados brutos no arquivo:
        sonarDataRawFile:write(""..global_x.."\t"..global_y.."\t"..global_theta.."\t"..(-math.pi/2).."\t"..dist.."\n")
        
        --Calcula o ponto (x,y) em funçao de global_x, global_y, global_theta e dist:
        x = 0 -- ???
        y = 0 -- ???
        
        --Grava o ponto no arquivo:
        sonarDataXYFile:write(""..x.."\t"..y.."\n")
    end
    --]]
end

---------------------------------------------------------------
-- Controle de Movimento
---------------------------------------------------------------

function nextGoal()
    g=g+1;
    if (g>table.getn(goals)) then
        g = 1
    end
    goal = goals[g]
    state=0
    simAuxiliaryConsolePrint(console, "Next goal: ("..goal[1]..","..goal[2]..","..math.deg(goal[3]).."\n")
end

function to_positive_angle(angle)

   angle = math.fmod(angle, 2*math.pi);
   while(angle < 0) do
     angle = angle + 2*math.pi;
   end

   return angle;
end

function to180range(angle)

   angle = math.fmod(angle, 2*math.pi);
       if (angle<-math.pi) then
        angle = angle + 2*math.pi
    elseif (angle>math.pi) then
        angle = angle - 2*math.pi
    end

   return angle;
end

function smallestAngleDiff( target, source )
   local a = to_positive_angle(target) - to_positive_angle(source)
   
   if (a > math.pi) then
      a = a - 2*math.pi
   elseif (a < -math.pi) then
      a = a + 2*math.pi
   end
   
   return a
end

function motionControl(goal)
    --Localizacao atual do robo [x, y, z]
    pos=simGetObjectPosition(ddRobot,-1) 
    --Angulo atual do robo
    orientation = simGetObjectOrientation(ddRobot,-1,pos) 
    theta = orientation[3]
    
    --Computar dx, dy, rho, alpha e beta
    dx = goal[1] - pos[1]
    dy = goal[2] - pos[2]
    dtheta = smallestAngleDiff(goal[3], theta)
    rho = math.sqrt(dx*dx + dy*dy)
    
    atg = math.atan2(dy, dx)
    atg = to180range(atg)
    alpha = smallestAngleDiff(atg,theta)
    alpha = to180range(alpha)
    beta = goal[3]-theta - alpha
    beta = to180range(beta)

    v = k_rho*rho

    if (v<0.2) then v=0.2 end

    omega = k_alpha*alpha + k_beta*(beta)

    wR = v + l*omega
    wL = v - l*omega

    phiR = wR/r
    phiL = wL/r

    -- o controle eh instavel com rho ~= 0
    if (rho<0.05 or state>0) then
        state = 1
        -- faz o robo apenas girar para corrigir theta
        if (math.abs(dtheta)>0.5) then
            phiR = 2*dtheta -- proporcionalmente ao que falta
            phiL = -2*dtheta
        else
            phiR = 0
            phiL = 0
            state=2
        end
    end

    return phiR, phiL, rho
end

if (sim_call_type==sim_childscriptcall_actuation) then
    --Limpa a janela do console
    --simAuxiliaryConsolePrint(console, NULL)

    if (state==2) then
        nextGoal()
    end

    phiR, phiL, rho = motionControl(goal)

    --Faz o robo apenas girar
    --phiR = 0.5
    --phiL = -0.5

    speedRight = phiR
    speedLeft = phiL

    ---------------------------------------------------------------
    -- UI control:
    -- Se o controle pelo slider estiver ativado
    -- Le a velocidade desejada da interface
    -- e realiza uma transformação para o intervalo de valores desejados
    -- no caso, o intervalo após a transformação é [-50,50]
    if (useSlider) then
        speedLeft = (simGetUISlider(uiHandle,3) - 500)/10
        speedRight = (simGetUISlider(uiHandle,1) - 500)/10
        simSetUIButtonLabel(uiHandle,2,string.format("%.1f",speedLeft))
        simSetUIButtonLabel(uiHandle,5,string.format("%.1f",speedRight))
    end
    ---------------------------------------------------------------
    
    -- Seta a velocidade dos motores, em radianos
    simSetJointTargetVelocity(leftMotor,speedLeft)
    simSetJointTargetVelocity(rightMotor,speedRight)

    --Imprime variaveis no console:
    simAuxiliaryConsolePrint(console, "Goal: ("..goal[1]..","..goal[2]..","..math.deg(goal[3])..") state: "..state.."\n")
    --simAuxiliaryConsolePrint(console, "Posicao: ("..pos[1]..","..pos[2]..","..math.deg(orientation[3])..")\n")
    --simAuxiliaryConsolePrint(console, "dx: "..dx.."  dy: "..dy.." dtheta: "..math.deg(dtheta).."\n")

    --simAuxiliaryConsolePrint(console, "alphaO: "..math.deg(atg).."  atg: "..math.deg(atg).."\n")
    --simAuxiliaryConsolePrint(console, "rho: "..rho.."    theta: "..math.deg(theta).."  alpha: "..math.deg(alpha).." beta: "..math.deg(beta).."\n")
    --simAuxiliaryConsolePrint(console, "left: "..speedLeft.."  right: "..speedRight.."\n")
end

if (sim_call_type==sim_childscriptcall_cleanup) then

    -- Fecha os arquivos
    sonarDataRawFile:close()
    sonarDataXYFile:close()
end
