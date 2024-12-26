package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.IPlayer;
import edu.upc.epsevg.prop.hex.IAuto;
import edu.upc.epsevg.prop.hex.PlayerMove;
import edu.upc.epsevg.prop.hex.PlayerType;
import edu.upc.epsevg.prop.hex.SearchType;
import java.awt.Point;
import java.util.*;

/**
 * Implementació d'un jugador que utilitza l'algorisme Minimax amb poda alfa-beta i taules de transposició
 * per decidir les seves jugades. Aquest jugador està optimitzat per ordenar moviments segons una heurística
 * personalitzada.
 * Permet l'ús de profunditat fixa o Iterative Deepening (ID). Es gestiona també una taula de transposició
 * per accelerar la cerca i evitar explorar estats repetits innecessàriament.
 * 
 * @author Sergi i Oscar
 */
public class PlayerMinimax implements IPlayer, IAuto {

    // Profunditat màxima de la cerca Minimax.
    public int profunditat;
    public String name;
    public PlayerType myColor;
    //comptador de nodes visitats
    public long nodesVisited;
    public static final int MAXIM = 1000000;
    //Indica si fem servir o no IDS.
    public boolean useIterativeDeepening;
    public volatile boolean stopSearch;//timeout
    public Map<Long, TranspositionEntry> transpositionTable;
    public static long[][] table;

    /**
     * Constructor principal del jugador Minimax.
     *
     * @param name Nom del jugador.
     * @param profunditat Profunditat màxima que s'emprarà en la cerca.
     * @param useIterativeDeepening Indica si emprarem ID en lloc de profunditat fixa.
     */
    public PlayerMinimax(String name, int profunditat, boolean useIterativeDeepening) {
        this.name = name;
        this.profunditat = profunditat;
        this.useIterativeDeepening = useIterativeDeepening;
        this.transpositionTable = new HashMap<>();
        this.stopSearch = false;
    }

    /**
     *
     * @param name Nom del jugador.
     * @param profunditat Profunditat màxima de cerca.
     */
    public PlayerMinimax(String name, int profunditat) {
        this(name, profunditat, false);
    }

    /**
     * Funció principal que es crida en el moment de fer un moviment. Retorna la jugada
     * escollida mitjançant l'algorisme Minimax.
     *
     * @param s Estat actual del tauler de joc.
     * @return La jugada seleccionada.
     */
    @Override
    public PlayerMove move(HexGameStatus s) {
        nodesVisited = 0;
        stopSearch = false;
        myColor = s.getCurrentPlayer();

        if (table == null) {
            initializeTable(s.getSize());
        }

        List<Point> possibleMoves = getLegalMoves(s);
        if (possibleMoves.isEmpty()) {
            return new PlayerMove(null, nodesVisited, profunditat, SearchType.MINIMAX);
        }

        // Escollim l'estratègia segons useIterativeDeepening
        if (!useIterativeDeepening) {
            return normalminimax(s, possibleMoves);
        } else {
            return IDS(s, possibleMoves);
        }
    }

    /**
     * Cerca Minimax a profunditat fixa (normal, sense IDS).
     * Ordena moviments segons la taula de transposició.
     *
     * @param s Estat actual del joc.
     * @param possibleMoves Llista de moviments legals a explorar.
     * @return El millor moviment trobat.
     */
    public PlayerMove normalminimax(HexGameStatus s, List<Point> possibleMoves) {
        Point bestMove = null;
        int bestValue = Integer.MIN_VALUE;
        int alpha = Integer.MIN_VALUE;
        int beta = MAXIM;

        long stateHash = computeHash(s);

        // Reordenar moviments
        TranspositionEntry entry = transpositionTable.get(stateHash);
        if (entry != null && entry.bestMove != null && possibleMoves.contains(entry.bestMove)) {
            possibleMoves.remove(entry.bestMove);
            possibleMoves.add(0, entry.bestMove);
        }

        // Explorem tots els moviments a la profunditat fixada
        for (Point move : possibleMoves) {
            HexGameStatus newState = applyMove(s, move);
            int value = minValue(newState, getOpponentColor(myColor), alpha, beta, profunditat - 1);
            if (value > bestValue) {
                bestValue = value;
                bestMove = move;
            }

            alpha = Math.max(alpha, bestValue);
            if (beta <= alpha || bestValue == MAXIM) {
                break;
            }
        }

        if (bestMove == null && !possibleMoves.isEmpty()) {
            bestMove = possibleMoves.get(0);
        }

        TranspositionTable(stateHash, bestValue, bestMove, profunditat);
        return new PlayerMove(bestMove, nodesVisited, profunditat, SearchType.MINIMAX);
    }

    /**
     * Cerca IDS des de profunditat 1 fins la profunditat indicada.
     * Si arriba un timeout enmig, ens quedem amb la millor jugada trobada fins llavors.
     *
     * @param s Estat actual del joc.
     * @param baseMoves Moviments legals de base (són els mateixos a cada iteració).
     * @return El millor moviment trobat a la darrera iteració completada.
     */
    public PlayerMove IDS(HexGameStatus s, List<Point> baseMoves) {
        Point bestMoveOverall = null; 

        for (int currentDepth = 1; !stopSearch; currentDepth++) {
            
            List<Point> possibleMoves = new ArrayList<>(baseMoves);
            long stateHash = computeHash(s);
            TranspositionEntry entry = transpositionTable.get(stateHash);
            if (entry != null && entry.bestMove != null && possibleMoves.contains(entry.bestMove)) {
                possibleMoves.remove(entry.bestMove);
                possibleMoves.add(0, entry.bestMove);
            }

            int alpha = Integer.MIN_VALUE;
            int beta = MAXIM;
            Point bestMoveThisDepth = null;
            int bestValueThisDepth = Integer.MIN_VALUE;

            for (Point move : possibleMoves) {
                if (stopSearch) break;

                HexGameStatus newState = applyMove(s, move);
                int value = minValue(newState, getOpponentColor(myColor), alpha, beta, currentDepth - 1);
                if (value > bestValueThisDepth) {
                    bestValueThisDepth = value;
                    bestMoveThisDepth = move;
                }

                alpha = Math.max(alpha, bestValueThisDepth);
                if (beta <= alpha || bestValueThisDepth == MAXIM) {
                    break;
                }
            }
            if (!stopSearch) {
                bestMoveOverall = bestMoveThisDepth;
            }
        }

        if (bestMoveOverall == null && !baseMoves.isEmpty()) {
            bestMoveOverall = baseMoves.get(0);
        }

        return new PlayerMove(bestMoveOverall, nodesVisited, profunditat, SearchType.MINIMAX);
    }

    /**
     * Retorna el nom del jugador.
     *
     * @return El nom del jugador, incloent la indicació [ID] si està en mode Iterative Deepening.
     */
    @Override
    public String getName() {
        return "Minimax(" + name + ")" + (useIterativeDeepening ? " [ID]" : "");
    }

    /**
     * Es crida quan es produeix un timeout i s'ha de parar la cerca immediatament.
     */
    @Override
    public void timeout() {
        stopSearch = true;
    }

    /**
     * Funció de l'algorisme Minimax (part MAX). Retorna el millor valor
     * possible per al jugador actual, explorant en profunditat fins que profunditat = 0
     * o si s'acaba la partida.
     *
     * @param s Estat del joc en el moment de cridar la funció.
     * @param currentPlayer El jugador que té el torn.
     * @param alpha L'actual límit inferior d'utilitat (poda alfa-beta).
     * @param beta L'actual límit superior d'utilitat (poda alfa-beta).
     * @param profunditat Profunditat restant de la cerca.
     * @return El millor valor (MAX) trobat per aquest node.
     */
    public int maxValue(HexGameStatus s, PlayerType currentPlayer, int alpha, int beta, int profunditat) {
        nodesVisited++;
        long stateHash = computeHash(s);
        TranspositionEntry ttEntry = transpositionTable.get(stateHash);
        if (ttEntry != null && ttEntry.depth >= profunditat) {
            if (ttEntry.flag == TranspositionEntry.EXACT) {
                return ttEntry.value;
            }
            else if (ttEntry.flag == TranspositionEntry.LOWERBOUND) {
                alpha = Math.max(alpha, ttEntry.value);
            }
            else if (ttEntry.flag == TranspositionEntry.UPPERBOUND) {
                beta = Math.min(beta, ttEntry.value);
            }
            if (alpha >= beta) {
                return ttEntry.value;
            }
        }

        if (profunditat == 0 || s.isGameOver()) {
            int val = evaluate(s, myColor);
            storeTT(stateHash, profunditat, val, alpha, beta, null);
            return val;
        }

        List<Point> possibleMoves = getLegalMoves(s);
        if (possibleMoves.isEmpty()) {
            int val = evaluate(s, myColor);
            storeTT(stateHash, profunditat, val, alpha, beta, null);
            return val;
        }

        if (ttEntry != null && ttEntry.bestMove != null && possibleMoves.contains(ttEntry.bestMove)) {
            possibleMoves.remove(ttEntry.bestMove);
            possibleMoves.add(0, ttEntry.bestMove);
        }

        int value = Integer.MIN_VALUE;
        Point bestMove = null;
        for (Point move : possibleMoves) {

            HexGameStatus newState = applyMove(s, move);
            int childValue = minValue(newState, getOpponentColor(currentPlayer), alpha, beta, profunditat - 1);
            if (childValue > value) {
                value = childValue;
                bestMove = move;
            }
            alpha = Math.max(alpha, value);
            if (alpha >= beta) {
                break;
            }
        }

        storeTT(stateHash, profunditat, value, alpha, beta, bestMove);
        return value;
    }

    /**
     * Implementa la part MIN de l'algorisme Minimax. Retorna el valor mínim possible
     * per al jugador contrari, explorant la profunditat fins a 0 o si la partida acaba.
     *
     * @param s Estat del joc en el moment de cridar la funció.
     * @param currentPlayer El jugador (color) que té el torn.
     * @param alpha L'actual límit inferior d'utilitat (poda alfa-beta).
     * @param beta L'actual límit superior d'utilitat (poda alfa-beta).
     * @param profunditat Profunditat restant de la cerca.
     * @return El millor valor (MIN) trobat per aquest node.
     */
    public int minValue(HexGameStatus s, PlayerType currentPlayer, int alpha, int beta, int profunditat) {
        nodesVisited++;

        long stateHash = computeHash(s);
        TranspositionEntry ttEntry = transpositionTable.get(stateHash);
        if (ttEntry != null && ttEntry.depth >= profunditat) {
            if (ttEntry.flag == TranspositionEntry.EXACT) {
                return ttEntry.value;
            }
            else if (ttEntry.flag == TranspositionEntry.LOWERBOUND) {
                alpha = Math.max(alpha, ttEntry.value);
            }
            else if (ttEntry.flag == TranspositionEntry.UPPERBOUND) {
                beta = Math.min(beta, ttEntry.value);
            }
            if (alpha >= beta) {
                return ttEntry.value;
            }
        }

        if (profunditat == 0 || s.isGameOver()) {
            int val = evaluate(s, myColor);
            storeTT(stateHash, profunditat, val, alpha, beta, null);
            return val;
        }

        List<Point> possibleMoves = getLegalMoves(s);
        if (possibleMoves.isEmpty()) {
            int val = evaluate(s, myColor);
            storeTT(stateHash, profunditat, val, alpha, beta, null);
            return val;
        }

        if (ttEntry != null && ttEntry.bestMove != null && possibleMoves.contains(ttEntry.bestMove)) {
            possibleMoves.remove(ttEntry.bestMove);
            possibleMoves.add(0, ttEntry.bestMove);
        }

        int value = Integer.MAX_VALUE;
        Point bestMove = null;
        for (Point move : possibleMoves) {
            HexGameStatus newState = applyMove(s, move);
            int childValue = maxValue(newState, getOpponentColor(currentPlayer), alpha, beta, profunditat - 1);
            if (childValue < value) {
                value = childValue;
                bestMove = move;
            }
            beta = Math.min(beta, value);
            if (beta <= alpha) {
                break;
            }
        }

        storeTT(stateHash, profunditat, value, alpha, beta, bestMove);
        return value;
    }

    /**
     * Funció d'avaluació heurística: calcula un valor (heurística) per a l'estat
     * del tauler donat, des de la perspectiva del jugador.
     *      Si el joc ha acabat, valor màxim o mínim depenent de qui hagi guanyat.
     *      bloqueig del camí del rival, etc.).
     *
     * @param s Estat actual del joc.
     * @param evaluatorColor Color del jugador que estem avaluant.
     * @return Valor enter que indica la “bona posició” per al jugador actual.
     */
    public int evaluate(HexGameStatus s, PlayerType evaluatorColor) {
    if (s.isGameOver()) {
        PlayerType winner = s.GetWinner();
        return (winner == evaluatorColor) ? MAXIM : -MAXIM;
    }
    PlayerType opponentColor = getOpponentColor(evaluatorColor);
    Dijkstra dijkstra = new Dijkstra();
    int distMe = dijkstra.calcularDistanciaMinima(s, evaluatorColor);
    int distOpp = dijkstra.calcularDistanciaMinima(s, opponentColor);

    if (distMe == 0) return MAXIM;
    if (distOpp == 0) return -MAXIM;
    
    double heuristic = 0;
    heuristic += distOpp * 2 - distMe;
    heuristic += bloquearCaminoDelOponente(s, opponentColor)*0.5;

    return (int) heuristic;
}
/**
 * Genera una llista de moviments de resposta per les amenaces de les fitxes de l'oponent.
 * Una resposta a una amenaça consisteix a identificar posicions adjacents a les fitxes de l'oponent.
 *
 * @param s Estat actual del joc.
 * @param player jugador que estem avaluant.
 * @return una llista de les posicions de les respostes a les amenaces.
 */
public List<Point> amenazas(HexGameStatus s, PlayerType player) {
    List<Point> respuestas = new ArrayList<>();
    List<Point> oponente = obtenerFichasPropias(s, getOpponentColor(player));

    for (Point piece : oponente) {
        Point[] directions = {
            new Point(-1, 0), new Point(1, 0), new Point(0, -1),
            new Point(0, 1), new Point(-1, 1), new Point(1, -1)
        };

        for (Point dir : directions) {
            Point amenazas = new Point(piece.x + dir.x, piece.y + dir.y);
            Point cont = new Point(amenazas.x, amenazas.y + dir.y);

            if (estaDentro(amenazas, s.getSize()) && s.getPos(amenazas.x, amenazas.y) == 0 &&
                estaDentro(cont, s.getSize()) && s.getPos(cont.x, cont.y) == 0) {
                respuestas.add(cont);
            }
        }
    }

    return respuestas;
}

    /**
     * Desa o actualitza la taula de transposició amb la informació (value, depth, flag)
     * per l'estat hash. Fa servir poda alfa-beta per determinar el flag.
     *
     * @param stateHash Hash de l'estat (Zobrist).
     * @param depth Profunditat a la qual hem calculat el valor.
     * @param value Valor del node.
     * @param alpha Valor alfa actual.
     * @param beta Valor beta actual.
     * @param bestMove Millor moviment trobat en aquest node.
     */
    public void storeTT(long stateHash, int depth, int value, int alpha, int beta, Point bestMove) {
        int flag;
        if (value <= alpha) {
            flag = TranspositionEntry.UPPERBOUND;
        } else if (value >= beta) {
            flag = TranspositionEntry.LOWERBOUND;
        } else {
            flag = TranspositionEntry.EXACT;
        }
        TranspositionEntry entry = new TranspositionEntry(value, depth, flag, bestMove);
        transpositionTable.put(stateHash, entry);
    }

    /**
     * Desa a la taula de transposició la informació del node arrel (flag = EXACT).
     *
     * @param stateHash Hash de l'estat.
     * @param bestValue Valor calculat.
     * @param bestMove Millor moviment trobat.
     * @param profunditat Profunditat de la cerca.
     */
    public void TranspositionTable(long stateHash, int bestValue, Point bestMove, int profunditat) {
        int flag = TranspositionEntry.EXACT;
        TranspositionEntry entry = new TranspositionEntry(bestValue, profunditat, flag, bestMove);
        transpositionTable.put(stateHash, entry);
    }

    /**
     * Retorna totes les fitxes pròpies presents al tauler.
     *
     * @param gameState Estat del joc.
     * @param player Jugador de qui volem trobar les fitxes.
     * @return Llista de coordenades on hi ha fitxes d'aquest jugador.
     */
    public ArrayList<Point> obtenerFichasPropias(HexGameStatus gameState, PlayerType player) {
        ArrayList<Point> fichas = new ArrayList<>();
        int size = gameState.getSize();
        int playerId = playerToId(player);

        for (int x = 0; x < size; x++) {
            for (int y = 0; y < size; y++) {
                if (gameState.getPos(x, y) == playerId) {
                    fichas.add(new Point(x, y));
                }
            }
        }
        return fichas;
    }

    /**
     * Verifica si una posició està dins dels límits del tauler.
     *
     * @param p Punt (x,y) a verificar.
     * @param size Mida del tauler (n x n).
     * @return Cert si està dins, fals si està fora.
     */
    public boolean estaDentro(Point p, int size) {
        return p.x >= 0 && p.y >= 0 && p.x < size && p.y < size;
    }

    /**
     * Retorna les celes transitables a la part superior.
     *
     * @param gameState Estat del joc.
     * @param playerId id del jugador.
     * @return Llista de punts transitables que comencen a la fila superior.
     */
    public List<Point> getTransitableTop(HexGameStatus gameState, int playerId) {
        List<Point> fuentes = new ArrayList<>();
        int size = gameState.getSize();
        for (int y = 0; y < size; y++) {
            if (gameState.getPos(0, y) != -playerId) {
                fuentes.add(new Point(0, y));
            }
        }
        return fuentes;
    }

    /**
     * Recupera les celes transitables des de la part esquerra del tauler.
     *
     * @param gameState Estat actual del joc.
     * @param playerId Identificador numèric del jugador (1 o -1).
     * @return Llista de punts transitables.
     */
    public List<Point> getTransitableLeft(HexGameStatus gameState, int playerId) {
        List<Point> fuentes = new ArrayList<>();
        int size = gameState.getSize();
        for (int x = 0; x < size; x++) {
            if (gameState.getPos(x, 0) != -playerId) {
                fuentes.add(new Point(x, 0));
            }
        }
        return fuentes;
    }

    /**
     * Calcula la distància mínima fins a la victòria per a un jugador, utilitzant
     * una versió optimitzada de Dijkstra.
     *
     * @param gameState Estat actual del joc.
     * @param p Jugador de qui volem la distància a la seva condició de victòria.
     * @return Distància mínima en "caselles" per connectar el seu camí. Si no hi ha camí, valor gran (999999).
     */
    public int calcularDistanciaMinima(HexGameStatus gameState, PlayerType p) {
        int size = gameState.getSize();
        int playerId = (p == PlayerType.PLAYER1) ? 1 : -1;

        List<Point> fuentes = (p == PlayerType.PLAYER1)
                            ? getTransitableTop(gameState, playerId)
                            : getTransitableLeft(gameState, playerId);

        if (fuentes.isEmpty()) return 999999;

        int[][] dist = new int[size][size];
        for (int i = 0; i < size; i++) {
            Arrays.fill(dist[i], Integer.MAX_VALUE);
        }

        PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingInt(n -> n.distance));

        for (Point f : fuentes) {
            int cost = getCost(gameState, f.x, f.y, playerId);
            if (cost != Integer.MAX_VALUE) {
                dist[f.x][f.y] = cost;
                pq.add(new Node(f.x, f.y, cost));
            }
        }
        
        Point[] directions = {
            new Point(0, 1), new Point(1, 0), new Point(1, -1),
            new Point(0, -1), new Point(-1, 0), new Point(-1, 1)
        };

        while (!pq.isEmpty()) {
            Node current = pq.poll();
            if (current.distance > dist[current.x][current.y]) continue;

            if ((p == PlayerType.PLAYER1 && current.x == size - 1) ||
                (p == PlayerType.PLAYER2 && current.y == size - 1)) {
                return current.distance;
            }

            for (Point d : directions) {
                int nx = current.x + d.x;
                int ny = current.y + d.y;
                if (nx < 0 || ny < 0 || nx >= size || ny >= size) continue;

                int c = getCost(gameState, nx, ny, playerId);
                if (c == Integer.MAX_VALUE) continue;

                int newDist = dist[current.x][current.y] + c;
                if (newDist < dist[nx][ny]) {
                    dist[nx][ny] = newDist;
                    pq.add(new Node(nx, ny, newDist));
                }
            }
        }
        return 999999;
    }

    /**
     * Determina el cost d'avançar a la posició (x,y), depenent de si la casella
     * és del jugador, està buida o és de l'oponent.
     *
     * @param gameState Estat actual del joc.
     * @param x Coordenada x de la casella.
     * @param y Coordenada y de la casella.
     * @param playerId 1 per a PLAYER1 o -1 per a PLAYER2.
     * @return 0 si la casella és del jugador, 1 si és buida, Integer.MAX_VALUE si és de l'oponent.
     */
    public int getCost(HexGameStatus gameState, int x, int y, int playerId) {
        int cell = gameState.getPos(x, y);
        if (cell == playerId) return 0;
        if (cell == 0) return 1;
        return (cell == -playerId) ? Integer.MAX_VALUE : 1;
    }

    /**
     * Afegeix penalització si el rival ja té un camí curt cap a la victòria.
     * Com més curt sigui el seu camí, més gran la penalització.
     *
     * @param gameState Estat actual del joc.
     * @param opponent Jugador rival.
     * @return Un valor negatiu proporcionalment més gran si la distància del rival és més petita.
     */
    public int bloquearCaminoDelOponente(HexGameStatus gameState, PlayerType opponent) {
        int distOpponent = calcularDistanciaMinima(gameState, opponent);
        if (distOpponent < 999999) {
            return -(1000 / (distOpponent + 1)); //+1 per evitar dividir entre 0
        }
        return 0;
    }

    /**
     * Converteix un PlayerType al seu id numèric (1 o -1).
     *
     * @param player El color del jugador.
     * @return 1 si és PLAYER1, -1 si és PLAYER2.
     */
    public int playerToId(PlayerType player) {
        return (player == PlayerType.PLAYER1) ? 1 : -1;
    }

    /**
     * Retorna el color contrari (PLAYER1 -> PLAYER2, i viceversa).
     *
     * @param color El color actual.
     * @return El color oposat.
     */
    public PlayerType getOpponentColor(PlayerType color) {
        return (color == PlayerType.PLAYER1) ? PlayerType.PLAYER2 : PlayerType.PLAYER1;
    }

    /**
     * Retorna tots els moviments legals (caselles buides) del tauler, ordenats
     * per un factor de centralitat (moviments més propers al centre primer).
     *
     * @param s Estat del joc.
     * @return Llista de moviments legals, ordenada.
     */
    public List<Point> getLegalMoves(HexGameStatus s) {
        List<Point> moves = new ArrayList<>();
        List<Point> threatResponses = amenazas(s, myColor);
        int size = s.getSize();
        Point center = new Point(size / 2, size / 2);

        moves.addAll(threatResponses);
        
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                if (s.getPos(i, j) == 0) {
                    moves.add(new Point(i, j));
                }
            }
        }

        moves.sort(Comparator.comparingDouble(move -> -centralityFactor(move, center)));


        return moves;
    }


    /**
     * Calcula un factor de centralitat per a la posició p, segons la distància
     * euclidiana al centre. Com més a prop del centre, més gran és el factor.
     *
     * @param p Punt objectiu.
     * @param center Punt central del tauler.
     * @return Un valor positiu més gran si p és a prop del centre.
     */
    public double centralityFactor(Point p, Point center) {
        return 1.0 / (Math.sqrt(Math.pow(p.x - center.x, 2) + Math.pow(p.y - center.y, 2)) + 1);
    }

    /**
     * Aplica un moviment (col·loca una fitxa) i retorna un nou estat resultant.
     *
     * @param s Estat original del joc.
     * @param move Moviment que volem fer.
     * @return Un nou HexGameStatus resultant d'haver col·locat la fitxa.
     */
    public HexGameStatus applyMove(HexGameStatus s, Point move) {
        HexGameStatus newState = new HexGameStatus(s);
        newState.placeStone(move);
        return newState;
    }

    /**
     * Inicialitza la taula per al Hashing. Assigna un valor aleatori 
     * a cada (casella, jugador) per poder distingir els estats.
     *
     * @param size Mida n del tauler (n x n).
     */
    public void initializeTable(int size) {
        Random rand = new Random(123456789);
        table = new long[size * size][2];
        for (int i = 0; i < size * size; i++) {
            for (int j = 0; j < 2; j++) {
                table[i][j] = rand.nextLong();
            }
        }
    }

    /**
     * Calcula el hash de l'estat actual del tauler.
     *
     * @param s Estat del joc.
     * @return Un long representant l'hash d'aquest estat.
     */
    public long computeHash(HexGameStatus s) {
        long h = 0L;
        int size = s.getSize();
        for (int x = 0; x < size; x++) {
            for (int y = 0; y < size; y++) {
                int val = s.getPos(x, y);
                if (val != 0) {
                    int index = x * size + y;
                    int playerIndex = (val == 1) ? 0 : 1; 
                    h ^= table[index][playerIndex];
                }
            }
        }
        return h;
    }

    /**
     * Classe interna per guardar entrades de la taula de transposició.
     * Emmagatzema informació de valor, profunditat, flag (EXACT, LOWERBOUND, UPPERBOUND),
     * i el possible millor moviment associat.
     */
    public static class TranspositionEntry {
        public static final int EXACT = 0;
        public static final int LOWERBOUND = 1;
        public static final int UPPERBOUND = 2;

        int value;
        int depth;
        int flag;
        Point bestMove;

        /**
         * Constructor de l'entrada de la taula de transposició.
         * 
         * @param value Valor a emmagatzemar.
         * @param depth Profunditat a la qual s'ha calculat.
         * @param flag Marca (EXACT, LOWERBOUND o UPPERBOUND).
         * @param bestMove El millor moviment trobat, si escau.
         */
        TranspositionEntry(int value, int depth, int flag, Point bestMove) {
            this.value = value;
            this.depth = depth;
            this.flag = flag;
            this.bestMove = bestMove;
        }
    }

    /**
     * Node utilitzat per Dijkstra. Guarda posició (x,y) i distància fins a la font.
     */
    public static class Node implements Comparable<Node> {
        int x, y, distance;

        /**
         * Constructor del node per Dijkstra.
         *
         * @param x Coordenada x.
         * @param y Coordenada y.
         * @param dist Distància des de la font fins a aquest node.
         */
        Node(int x, int y, int dist) {
            this.x = x;
            this.y = y;
            this.distance = dist;
        }

        /**
         * Compara segons la distància (per fer PriorityQueue).
         *
         * @param other L'altre node a comparar.
         * @return Comparació lexicogràfica per distance.
         */
        @Override
        public int compareTo(Node other) {
            return Integer.compare(this.distance, other.distance);
        }
    }
}