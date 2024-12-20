package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.IPlayer;
import edu.upc.epsevg.prop.hex.IAuto;
import edu.upc.epsevg.prop.hex.PlayerMove;
import edu.upc.epsevg.prop.hex.PlayerType;
import edu.upc.epsevg.prop.hex.SearchType;
import static edu.upc.epsevg.prop.hex.players.HexHeuristica.PESO_CONECTIVIDAD;
import static edu.upc.epsevg.prop.hex.players.HexHeuristica.PESO_FLEXIBILIDAD;
import static edu.upc.epsevg.prop.hex.players.HexHeuristica.PESO_GRUPOS_AISLADOS;
import static edu.upc.epsevg.prop.hex.players.HexHeuristica.PESO_MOVILIDAD;
import static edu.upc.epsevg.prop.hex.players.HexHeuristica.PESO_PUENTES_VIRTUALES;
import java.awt.Point;
import java.util.*;

/**
 * Jugador Minimax con poda alfa-beta y tabla de transposición.
 */
public class PlayerMinimax implements IPlayer, IAuto {
    private int depth;
    private String name;
    private PlayerType myColor;
    private long nodesVisited;
    private long nodesPruned;
    private static final int MAXIM = 1000000;
    private boolean debug = false;
    

    // Direcciones hex
    private static final int[][] HEX_DIRECTIONS = {
        {-1,0}, {-1,1}, {0,-1}, {0,1}, {1,-1}, {1,0}
    };

    // Tabla de transposición
    // Llave: hash del estado. Valor: entrada de transposición
    private Map<Long, TranspositionEntry> transpositionTable;

    // Zobrist hashing
    private static long[][] zobristTable; 
    // Asumiendo dos jugadores: 
    // PLAYER1 ocupación = 1, PLAYER2 ocupación = -1
    // Podemos mapear: PLAYER1 -> índice 0, PLAYER2 -> índice 1
    // Para vacío no se XORea nada.
    // zobristTable[x][y][playerIndex]

    public PlayerMinimax(String name, int depth) {
        this.name = name;
        this.depth = depth;
        this.transpositionTable = new HashMap<>();
    }

    @Override
    public PlayerMove move(HexGameStatus s) {
        nodesVisited = 0;
        nodesPruned = 0;
        myColor = s.getCurrentPlayer();

        if (zobristTable == null) {
            initializeZobristTable(s.getSize());
        }

        if (debug) {
            System.out.println("Color actual del jugador: " + myColor);
        }

        List<Point> possibleMoves = getLegalMoves(s);
        if (possibleMoves.isEmpty()) {
            // No hay movimientos
            return new PlayerMove(null, nodesVisited, depth, SearchType.MINIMAX);
        }

        Point bestMove = null;
        int bestValue = Integer.MIN_VALUE;
        int alpha = Integer.MIN_VALUE;
        int beta = MAXIM;

        long stateHash = computeHash(s);

        // Si hay una transposition entry, se puede priorizar el mejor hijo
        TranspositionEntry entry = transpositionTable.get(stateHash);
        if (entry != null && entry.bestMove != null && possibleMoves.contains(entry.bestMove)) {
            // Mover el mejor movimiento anterior al frente para priorizarlo
            possibleMoves.remove(entry.bestMove);
            possibleMoves.add(0, entry.bestMove);
        }

        for (Point move : possibleMoves) {
            HexGameStatus newState = applyMove(s, move);
            int value = minValue(newState, getOpponentColor(myColor), alpha, beta, depth - 1);
            if (value > bestValue) {
                bestValue = value;
                bestMove = move;
            }

            // Poda
            alpha = Math.max(alpha, bestValue);
            if (beta <= alpha || bestValue == MAXIM) {
                break;
            }
        }

        // Fallback si no encuentra nada (poco probable)
        if (bestMove == null && !possibleMoves.isEmpty()) {
            bestMove = possibleMoves.get(0);
        }

        // Guardar en transposition table
        saveToTranspositionTable(stateHash, bestValue, bestMove, depth);

        return new PlayerMove(bestMove, nodesVisited, depth, SearchType.MINIMAX);
    }

    @Override
    public String getName() {
        return "Minimax(" + name + ")";
    }

    @Override
    public void timeout() {
        if (debug) {
            System.out.println("Timeout occurred.");
        }
    }

    private int maxValue(HexGameStatus s, PlayerType currentPlayer, int alpha, int beta, int depth) {
        nodesVisited++;

        long stateHash = computeHash(s);
        // Consulta de transposition table
        TranspositionEntry ttEntry = transpositionTable.get(stateHash);
        if (ttEntry != null && ttEntry.depth >= depth) {
            // Podemos usar el valor guardado si es apropiado
            if (ttEntry.flag == TranspositionEntry.EXACT) {
                return ttEntry.value;
            } else if (ttEntry.flag == TranspositionEntry.LOWERBOUND) {
                alpha = Math.max(alpha, ttEntry.value);
            } else if (ttEntry.flag == TranspositionEntry.UPPERBOUND) {
                beta = Math.min(beta, ttEntry.value);
            }
            if (alpha >= beta) {
                return ttEntry.value;
            }
        }

        if (depth == 0 || s.isGameOver()) {
            int val = evaluate(s, myColor);
            storeTT(stateHash, depth, val, alpha, beta, null);
            return val;
        }

        List<Point> possibleMoves = getLegalMoves(s);
        if (possibleMoves.isEmpty()) {
            int val = evaluate(s, myColor);
            storeTT(stateHash, depth, val, alpha, beta, null);
            return val;
        }

        // Si hay TT con bestMove
        if (ttEntry != null && ttEntry.bestMove != null && possibleMoves.contains(ttEntry.bestMove)) {
            // Priorizar bestMove
            possibleMoves.remove(ttEntry.bestMove);
            possibleMoves.add(0, ttEntry.bestMove);
        }

        int value = Integer.MIN_VALUE;
        Point bestMove = null;
        for (Point move : possibleMoves) {
            HexGameStatus newState = applyMove(s, move);
            int childValue = minValue(newState, getOpponentColor(currentPlayer), alpha, beta, depth - 1);
            if (childValue > value) {
                value = childValue;
                bestMove = move;
            }

            alpha = Math.max(alpha, value);
            if (alpha >= beta) {
                break;
            }
        }

        storeTT(stateHash, depth, value, alpha, beta, bestMove);
        return value;
    }

    private int minValue(HexGameStatus s, PlayerType currentPlayer, int alpha, int beta, int depth) {
        nodesVisited++;

        long stateHash = computeHash(s);
        // Consulta de transposition table
        TranspositionEntry ttEntry = transpositionTable.get(stateHash);
        if (ttEntry != null && ttEntry.depth >= depth) {
            if (ttEntry.flag == TranspositionEntry.EXACT) {
                return ttEntry.value;
            } else if (ttEntry.flag == TranspositionEntry.LOWERBOUND) {
                alpha = Math.max(alpha, ttEntry.value);
            } else if (ttEntry.flag == TranspositionEntry.UPPERBOUND) {
                beta = Math.min(beta, ttEntry.value);
            }
            if (alpha >= beta) {
                return ttEntry.value;
            }
        }

        if (depth == 0 || s.isGameOver()) {
            int val = evaluate(s, myColor);
            storeTT(stateHash, depth, val, alpha, beta, null);
            return val;
        }

        List<Point> possibleMoves = getLegalMoves(s);
        if (possibleMoves.isEmpty()) {
            int val = evaluate(s, myColor);
            storeTT(stateHash, depth, val, alpha, beta, null);
            return val;
        }

        // Si hay TT con bestMove
        if (ttEntry != null && ttEntry.bestMove != null && possibleMoves.contains(ttEntry.bestMove)) {
            // Priorizamos el bestMove anterior
            possibleMoves.remove(ttEntry.bestMove);
            possibleMoves.add(0, ttEntry.bestMove);
        }

        int value = Integer.MAX_VALUE;
        Point bestMove = null;
        for (Point move : possibleMoves) {
            HexGameStatus newState = applyMove(s, move);
            int childValue = maxValue(newState, getOpponentColor(currentPlayer), alpha, beta, depth - 1);
            if (childValue < value) {
                value = childValue;
                bestMove = move;
            }

            beta = Math.min(beta, value);
            if (beta <= alpha) {
                break;
            }
        }

        storeTT(stateHash, depth, value, alpha, beta, bestMove);
        return value;
    }
    private int evaluarFlexibilidadRutas(HexGameStatus gameState, PlayerType player) {
        int size = gameState.getSize();
        int playerId = playerToId(player);
        int flexibilidad = 0;

        // Definir movimientos posibles (6 direcciones hexagonales).
        Point[] directions = {
            new Point(0, 1), new Point(1, 0), new Point(1, -1),
            new Point(0, -1), new Point(-1, 0), new Point(-1, 1)
        };

        // Recuperar celdas de inicio y fin según el jugador.
        List<Point> fuentes = (player == PlayerType.PLAYER1) ? getTransitableTop(gameState, playerId) : getTransitableLeft(gameState, playerId);
        List<Point> destinos = (player == PlayerType.PLAYER1) ? getTransitableBottom(gameState, playerId) : getTransitableRight(gameState, playerId);

        // Realizar búsquedas de rutas y contar celdas de destino alcanzables.
        Set<Point> alcanzables = new HashSet<>();

        for (Point fuente : fuentes) {
            alcanzables.addAll(bfsAlcanzables(gameState, fuente, playerId, destinos));
        }

        flexibilidad = alcanzables.size();

        // Convertir el número de celdas alcanzables en una puntuación.
        // Más celdas alcanzables indican mayor flexibilidad.
        if (flexibilidad >= size) {
            return 30; // Alta flexibilidad.
        } else if (flexibilidad >= size / 2) {
            return 20; // Buena flexibilidad.
        } else if (flexibilidad >= size / 4) {
            return 10; // Flexibilidad moderada.
        } else {
            return -10; // Baja flexibilidad.
        }
    }

    /**
     * Realiza una búsqueda en anchura (BFS) para determinar las celdas de destino alcanzables
     * desde una fuente dada.
     *
     * @param gameState Estado actual del juego.
     * @param fuente    Punto de inicio de la búsqueda.
     * @param playerId  Identificador numérico del jugador.
     * @param destinos  Lista de puntos que representan los destinos finales.
     * @return Conjunto de puntos alcanzables desde la fuente hacia los destinos.
     */
    private Set<Point> bfsAlcanzables(HexGameStatus gameState, Point fuente, int playerId, List<Point> destinos) {
        Set<Point> alcanzables = new HashSet<>();
        Queue<Point> queue = new LinkedList<>();
        boolean[][] visitado = new boolean[gameState.getSize()][gameState.getSize()];

        queue.add(fuente);
        visitado[fuente.x][fuente.y] = true;

        // Definir movimientos posibles (6 direcciones hexagonales).
        Point[] directions = {
            new Point(1, 0), new Point(0, 1), new Point(1, -1),
            new Point(-1, 1), new Point(-1, 0), new Point(0, -1)
        };

        while (!queue.isEmpty()) {
            Point actual = queue.poll();
            if (destinos.contains(actual)) {
                alcanzables.add(actual);
                continue; // No es necesario explorar más desde aquí.
            }

            for (Point d : directions) {
                int nx = actual.x + d.x;
                int ny = actual.y + d.y;
                Point vecino = new Point(nx, ny);
                if (estaDentro(vecino, gameState.getSize()) && !visitado[nx][ny]) {
                    int cell = gameState.getPos(nx, ny);
                    if (cell == playerId || cell == 0) { // Puede mover a celdas propias o vacías.
                        visitado[nx][ny] = true;
                        queue.add(vecino);
                    }
                }
            }
        }

        return alcanzables;
    }

    /**
     * Evalúa y cuenta los puentes virtuales para un jugador específico.
     *
     * @param gameState Estado actual del juego.
     * @param player    Jugador para el cual se evalúan los puentes virtuales.
     * @return Número de puentes virtuales detectados.
     */
    private int evaluarPuentesVirtuales(HexGameStatus gameState, PlayerType player) {
        ArrayList<Point> fichas = obtenerFichasPropias(gameState, player);
        int size = gameState.getSize();
        int playerId = playerToId(player);
        int puentes = 0;

        // Definir direcciones hexagonales.
        Point[] directions = {
            new Point(1, 0), new Point(0, 1), new Point(1, -1),
            new Point(-1, 1), new Point(-1, 0), new Point(0, -1)
        };

        // Utilizar un conjunto para evitar contar el mismo puente múltiples veces.
        Set<String> puentesDetectados = new HashSet<>();

        for (Point ficha : fichas) {
            for (Point dir : directions) {
                int nx1 = ficha.x + dir.x;
                int ny1 = ficha.y + dir.y;
                int nx2 = ficha.x + 2 * dir.x;
                int ny2 = ficha.y + 2 * dir.y;

                Point intermedio = new Point(nx1, ny1);
                Point destino = new Point(nx2, ny2);

                if (estaDentro(intermedio, size) && estaDentro(destino, size)) {
                    // Verificar si el intermedio está vacío y el destino pertenece al mismo jugador.
                    if (gameState.getPos(intermedio.x, intermedio.y) == 0 &&
                        gameState.getPos(destino.x, destino.y) == playerId) {

                        // Crear una clave única para el puente para evitar duplicados.
                        String clavePuente = crearClavePuente(ficha, destino);
                        if (!puentesDetectados.contains(clavePuente)) {
                            puentesDetectados.add(clavePuente);
                            puentes++;
                        }
                    }
                }
            }
        }

        return puentes;
    }
    private void storeTT(long stateHash, int depth, int value, int alpha, int beta, Point bestMove) {
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

    private void saveToTranspositionTable(long stateHash, int bestValue, Point bestMove, int depth) {
        int flag = TranspositionEntry.EXACT; // Del nodo raíz guardamos exacto
        TranspositionEntry entry = new TranspositionEntry(bestValue, depth, flag, bestMove);
        transpositionTable.put(stateHash, entry);
    }

    /**
     * Evalúa el estado enumerando caminos optimizados.
     */
    private int evaluate(HexGameStatus s, PlayerType evaluatorColor) {
    if (s.isGameOver()) {
        PlayerType winner = s.GetWinner();
        return (winner == myColor) ? MAXIM : -MAXIM;
    }

    PlayerType opponentColor = getOpponentColor(evaluatorColor);
    double heuristic = 0;

    // 1. Priorizar caminos hacia la victoria
    int distCurrent = calcularDistanciaMinima(s, evaluatorColor);
    int distOpponent = calcularDistanciaMinima(s, opponentColor);
    if (distCurrent == 0) return MAXIM; // Victoria asegurada.
    if (distOpponent == 0) return -MAXIM; // Derrota asegurada.
    heuristic += (distOpponent - distCurrent)*2; // Ponderación más alta.
    double evaluatorScore = calculateAllPathsScoreOptimized(s, evaluatorColor);
        double opponentScore = calculateAllPathsScoreOptimized(s, opponentColor);
       heuristic += (evaluatorScore - opponentScore)* 3;
    // 2. Evaluar flexibilidad de rutas
    int flexibilidad = evaluarFlexibilidadRutas(s, evaluatorColor) - evaluarFlexibilidadRutas(s, opponentColor);
    //heuristic += flexibilidad * 8;

    // 3. Puentes virtuales
    int puentesVirtuales = evaluarPuentesVirtuales(s, evaluatorColor) - evaluarPuentesVirtuales(s, opponentColor);
    //heuristic += puentesVirtuales * 10;

    // 4. Penalización por grupos aislados
    int penalizacionAislados = penalizarGruposAislados(s, evaluatorColor) - penalizarGruposAislados(s, opponentColor);
    //heuristic -= penalizacionAislados ;

    // 5. Movilidad
    int movilidad = calcularMovilidad(s, evaluatorColor) - calcularMovilidad(s, opponentColor);
    heuristic += movilidad * 1;

    // 6. Conectividad
    int conectividad = evaluarConectividad(s, evaluatorColor) - evaluarConectividad(s, opponentColor);
    heuristic += conectividad * 4;

    // 7. Bloqueo de caminos del oponente
    heuristic += bloquearCaminoDelOponente(s, opponentColor) ;

    return (int) heuristic;
}

     private ArrayList<Point> obtenerFichasPropias(HexGameStatus gameState, PlayerType player) {
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
      private int penalizarGruposAislados(HexGameStatus gameState, PlayerType currentPlayer) {
        ArrayList<Point> fichas = obtenerFichasPropias(gameState, currentPlayer);
        int size = gameState.getSize();
        HexHeuristica.UnionFind uf = new HexHeuristica.UnionFind(size * size);

        // Definir direcciones posibles (6 direcciones hexagonales).
        Point[] directions = {
            new Point(1, 0), new Point(0, 1), new Point(1, -1),
            new Point(-1, 1), new Point(-1, 0), new Point(0, -1)
        };

        // Unir fichas adyacentes.
        for (Point ficha : fichas) {
            int index1 = ficha.x * size + ficha.y;
            for (Point d : directions) {
                Point neighbor = new Point(ficha.x + d.x, ficha.y + d.y);
                if (estaDentro(neighbor, size) && gameState.getPos(neighbor.x, neighbor.y) == playerToId(currentPlayer)) {
                    int index2 = neighbor.x * size + neighbor.y;
                    uf.union(index1, index2);
                }
            }
        }

        // Agrupar fichas por sus raíces en Union-Find.
        Map<Integer, List<Point>> grupos = new HashMap<>();
        for (Point ficha : fichas) {
            int index = ficha.x * size + ficha.y;
            int root = uf.find(index);
            grupos.computeIfAbsent(root, k -> new ArrayList<>()).add(ficha);
        }

        // Determinar cuáles grupos están conectados a las fronteras del tablero.
        Set<Integer> gruposConectados = new HashSet<>();
        for (Map.Entry<Integer, List<Point>> entry : grupos.entrySet()) {
            boolean conectadoSuperior = false;
            boolean conectadoInferior = false;
            boolean conectadoIzquierda = false;
            boolean conectadoDerecha = false;

            for (Point ficha : entry.getValue()) {
                if (ficha.y == 0) conectadoIzquierda = true;
                if (ficha.y == size - 1) conectadoDerecha = true;
                if (ficha.x == 0) conectadoSuperior = true;
                if (ficha.x == size - 1) conectadoInferior = true;
            }

            // Para PLAYER1, conectarse de superior a inferior.
            // Para PLAYER2, conectarse de izquierda a derecha.
            if (currentPlayer == PlayerType.PLAYER1) {
                if (conectadoSuperior && conectadoInferior) {
                    gruposConectados.add(entry.getKey());
                }
            } else { // PLAYER2
                if (conectadoIzquierda && conectadoDerecha) {
                    gruposConectados.add(entry.getKey());
                }
            }
        }

        // Cualquier grupo que no esté en gruposConectados se considera aislado.
        int gruposAislados = 0;
        for (Map.Entry<Integer, List<Point>> entry : grupos.entrySet()) {
            if (!gruposConectados.contains(entry.getKey())) {
                gruposAislados++;
            }
        }

        return gruposAislados;
    }
       private String crearClavePuente(Point ficha1, Point ficha2) {
        // Ordenar las fichas para asegurar la unicidad de la clave.
        if (ficha1.x < ficha2.x || (ficha1.x == ficha2.x && ficha1.y <= ficha2.y)) {
            return ficha1.x + "," + ficha1.y + "-" + ficha2.x + "," + ficha2.y;
        } else {
            return ficha2.x + "," + ficha2.y + "-" + ficha1.x + "," + ficha1.y;
        }
    }
    /**
     * Verifica si una posición está dentro de los límites del tablero.
     *
     * @param p    Punto a verificar.
     * @param size Tamaño del tablero.
     * @return Verdadero si está dentro, falso en caso contrario.
     */
    private boolean estaDentro(Point p, int size) {
        return p.x >= 0 && p.y >= 0 && p.x < size && p.y < size;
    }
private List<Point> getTransitableTop(HexGameStatus gameState, int playerId) {
        List<Point> fuentes = new ArrayList<>();
        int size = gameState.getSize();
        for (int y = 0; y < size; y++) {
            if (gameState.getPos(0, y) != -playerId) {
                fuentes.add(new Point(0, y));
            }
        }
        return fuentes;
    }
 private List<Point> getTransitableBottom(HexGameStatus gameState, int playerId) {
        List<Point> fuentes = new ArrayList<>();
        int size = gameState.getSize();
        for (int y = 0; y < size; y++) {
            if (gameState.getPos(size - 1, y) != -playerId) {
                fuentes.add(new Point(size - 1, y));
            }
        }
        return fuentes;
    }

    /**
     * Recupera las celdas transitables desde la parte derecha del tablero para PLAYER2.
     *
     * @param gameState Estado actual del juego.
     * @param playerId Identificador numérico del jugador.
     * @return Lista de puntos transitables.
     */
    private List<Point> getTransitableRight(HexGameStatus gameState, int playerId) {
        List<Point> fuentes = new ArrayList<>();
        int size = gameState.getSize();
        for (int x = 0; x < size; x++) {
            if (gameState.getPos(x, size - 1) != -playerId) {
                fuentes.add(new Point(x, size - 1));
            }
        }
        return fuentes;
    }

    /**
     * Evalúa conexiones virtuales alrededor de una ficha específica para un jugador.
     * Considera patrones que pueden favorecer o perjudicar al jugador.
     *
     * @param gameState Estado actual del juego.
     * @param ficha     Ficha desde la cual se evalúan las conexiones.
     * @param player    Jugador para el cual se evalúan las conexiones.
     * @return Puntuación basada en las conexiones virtuales.
     */
    private int evaluarConexionesVirtuales(HexGameStatus gameState, Point ficha, PlayerType player) {
        int playerId = playerToId(player);
        // Definir direcciones únicas y relevantes (6 direcciones hexagonales).
        Point[] directions = {
            new Point(1, 0), new Point(0, 1), new Point(1, -1),
            new Point(-1, 1), new Point(-1, 0), new Point(0, -1)
        };
        int score = 0;

        for (Point d : directions) {
            Point neighbor = new Point(ficha.x + d.x, ficha.y + d.y);
            if (estaDentro(neighbor, gameState.getSize())) {
                if (gameState.getPos(neighbor.x, neighbor.y) == playerId) {
                    score += 20; // Conexión directa.
                } else if (gameState.getPos(neighbor.x, neighbor.y) == 0) {
                    // Verificar si hay una conexión potencial a través de celdas vacías.
                    score += 10;
                } else {
                    score -= 10; // Bloqueo potencial.
                }
            }
        }
        return score;
    }
    /**
     * Recupera las celdas transitables desde la parte izquierda del tablero para PLAYER2.
     *
     * @param gameState Estado actual del juego.
     * @param playerId Identificador numérico del jugador.
     * @return Lista de puntos transitables.
     */
    private List<Point> getTransitableLeft(HexGameStatus gameState, int playerId) {
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
     * Calcula la distancia mínima hasta la victoria para un jugador dado utilizando
     * una versión optimizada del algoritmo de Dijkstra.
     *
     * @param gameState Estado actual del juego.
     * @param p         Jugador para el cual se calcula la distancia.
     * @return Distancia mínima en "pasos" necesarias. Si no hay camino, retorna un valor grande.
     */
    private int calcularDistanciaMinima(HexGameStatus gameState, PlayerType p) {
        int size = gameState.getSize();
        int playerId = (p == PlayerType.PLAYER1) ? 1 : -1;

        // Crear lista de nodos fuente según el jugador.
        List<Point> fuentes = (p == PlayerType.PLAYER1) ? getTransitableTop(gameState, playerId) : getTransitableLeft(gameState, playerId);

        // Si no hay fuentes transitables, retornar distancia muy grande.
        if (fuentes.isEmpty()) return 999999;

        // Inicializar matrices de distancia.
        int[][] dist = new int[size][size];
        for (int i = 0; i < size; i++) {
            Arrays.fill(dist[i], Integer.MAX_VALUE);
        }

        // PriorityQueue optimizada para Dijkstra.
        PriorityQueue<HexHeuristica.Node> pq = new PriorityQueue<>(Comparator.comparingInt(n -> n.distance));

        // Inicializar distancias desde las fuentes.
        for (Point f : fuentes) {
            int cost = getCost(gameState, f.x, f.y, playerId);
            if (cost != Integer.MAX_VALUE) {
                dist[f.x][f.y] = cost;
                pq.add(new HexHeuristica.Node(f.x, f.y, cost));
            }
        }

        // Definir movimientos posibles en Hex.
        Point[] directions = {
            new Point(0, 1), new Point(1, 0), new Point(1, -1),
            new Point(0, -1), new Point(-1, 0), new Point(-1, 1)
        };

        // Ejecutar Dijkstra.
        while (!pq.isEmpty()) {
            HexHeuristica.Node current = pq.poll();

            // Ignorar nodos ya procesados con menor distancia.
            if (current.distance > dist[current.x][current.y]) continue;

            // Verificar si se ha llegado al borde contrario.
            if ((p == PlayerType.PLAYER1 && current.x == size - 1) ||
                (p == PlayerType.PLAYER2 && current.y == size - 1)) {
                return current.distance;
            }

            // Explorar vecinos.
            for (Point d : directions) {
                int nx = current.x + d.x;
                int ny = current.y + d.y;
                if (nx < 0 || ny < 0 || nx >= size || ny >= size) continue;

                int c = getCost(gameState, nx, ny, playerId);
                if (c == Integer.MAX_VALUE) continue; // No se puede pasar por el rival.

                int newDist = dist[current.x][current.y] + c;
                if (newDist < dist[nx][ny]) {
                    dist[nx][ny] = newDist;
                    pq.add(new HexHeuristica.Node(nx, ny, newDist));
                }
            }
        }

        // No se encontró conexión.
        return 999999;
    }
    private int getCost(HexGameStatus gameState, int x, int y, int playerId) {
        int cell = gameState.getPos(x, y);
        if (cell == playerId) return 0;
        if (cell == 0) return 1;
        // cell == -playerId => rival
        return (cell == -playerId) ? Integer.MAX_VALUE : 1;
    }
    
    private double calculateAllPathsScoreOptimized(HexGameStatus s, PlayerType color) {
        List<List<Point>> paths = findAllPathsOptimized(s, color);
        double score = 0.0;
            score = paths.size();
        return score;
    }
     private int bloquearCaminoDelOponente(HexGameStatus gameState, PlayerType opponent) {
        int distOpponent = calcularDistanciaMinima(gameState, opponent);
        if (distOpponent < 999999) {
            // Penalización inversamente proporcional a la distancia del oponente.
            // Cuanto menor sea la distancia, mayor la penalización.
            return -(1000 / (distOpponent + 1)); // +1 para evitar división por cero.
        }
        return 0;
    }

    /**
     * Calcula la movilidad de un jugador, es decir, la cantidad de celdas vacías adyacentes
     * a las fichas del jugador. Una mayor movilidad indica una mayor flexibilidad en las
     * opciones de movimiento.
     *
     * @param gameState Estado actual del juego.
     * @param player    Jugador para el cual se calcula la movilidad.
     * @return Número total de celdas vacías adyacentes a las fichas del jugador.
     */
    private int calcularMovilidad(HexGameStatus gameState, PlayerType player) {
        ArrayList<Point> fichas = obtenerFichasPropias(gameState, player);
        int movilidad = 0;
        int size = gameState.getSize();

        // Definir direcciones posibles (6 direcciones hexagonales).
        Point[] directions = {
            new Point(1, 0), new Point(0, 1), new Point(1, -1),
            new Point(-1, 1), new Point(-1, 0), new Point(0, -1)
        };

        // Utilizar un conjunto para evitar contar la misma celda vacía múltiples veces.
        Set<Point> celdasVacias = new HashSet<>();
        for (Point ficha : fichas) {
            for (Point d : directions) {
                Point neighbor = new Point(ficha.x + d.x, ficha.y + d.y);
                if (estaDentro(neighbor, size) && gameState.getPos(neighbor.x, neighbor.y) == 0) {
                    celdasVacias.add(neighbor);
                }
            }
        }

        movilidad = celdasVacias.size();
        return movilidad;
    }
 private int playerToId(PlayerType player) {
        return (player == PlayerType.PLAYER1) ? 1 : -1;
    }
    /**
     * Evalúa la conectividad de un jugador, es decir, cuántas agrupaciones conectadas
     * de fichas tiene el jugador. Menos agrupaciones indican una mejor conectividad.
     *
     * @param gameState Estado actual del juego.
     * @param player    Jugador para el cual se evalúa la conectividad.
     * @return Puntuación basada en la conectividad. Menos agrupaciones resultan en una mayor puntuación.
     */
    private int evaluarConectividad(HexGameStatus gameState, PlayerType player) {
        ArrayList<Point> fichas = obtenerFichasPropias(gameState, player);
        int size = gameState.getSize();
        HexHeuristica.UnionFind uf = new HexHeuristica.UnionFind(size * size);

        // Definir direcciones posibles (6 direcciones hexagonales).
        Point[] directions = {
            new Point(1, 0), new Point(0, 1), new Point(1, -1),
            new Point(-1, 1), new Point(-1, 0), new Point(0, -1)
        };

        // Unir fichas adyacentes.
        for (Point ficha : fichas) {
            int index1 = ficha.x * size + ficha.y;
            for (Point d : directions) {
                Point neighbor = new Point(ficha.x + d.x, ficha.y + d.y);
                if (estaDentro(neighbor, size) && gameState.getPos(neighbor.x, neighbor.y) == playerToId(player)) {
                    int index2 = neighbor.x * size + neighbor.y;
                    uf.union(index1, index2);
                }
            }
        }

        // Contar el número de grupos conectados.
        Set<Integer> grupos = new HashSet<>();
        for (Point ficha : fichas) {
            int index = ficha.x * size + ficha.y;
            grupos.add(uf.find(index));
        }

        // Menos grupos indican mejor conectividad.
        // Convertimos el número de grupos en una puntuación donde menos grupos son mejores.
        int numeroGrupos = grupos.size();
        if (numeroGrupos == 1) {
            return 30; // Excelente conectividad.
        } else if (numeroGrupos <= 3) {
            return 20; // Buena conectividad.
        } else if (numeroGrupos <= 5) {
            return 10; // Conectividad aceptable.
        } else {
            return -10; // Pobre conectividad.
        }
    }
    private List<List<Point>> findAllPathsOptimized(HexGameStatus s, PlayerType color) {
        int size = s.getSize();
        int[] nodes = getVirtualNodes(color, size);
        int startNode = nodes[0];
        int endNode = nodes[1];

        // Construir grafo
        List<List<Integer>> graph = buildGraphForPaths(s, color);

        // Calcular distMin con Dijkstra
        int distMin = calculateShortestDistanceDijkstra(graph, startNode, endNode);
        if (distMin == Integer.MAX_VALUE) {
            return new ArrayList<>();
        }
        int limitDist = distMin+1;

        // Filtrar grafo
        filterGraph(graph, startNode, endNode);

        List<List<Integer>> allPaths = new ArrayList<>();
        Set<Integer> visited = new HashSet<>();
        List<Integer> currentPath = new ArrayList<>();

        findAllPathsRecursiveOptimized(graph, startNode, endNode, visited, currentPath, allPaths, limitDist, size);

        return convertPathsToPoints(allPaths, size);
    }

    private void findAllPathsRecursiveOptimized(List<List<Integer>> graph, int current, int endNode,
                                                Set<Integer> visited, List<Integer> currentPath,
                                                List<List<Integer>> allPaths, int limitDist, int size) {
        final int MAX_PATHS = 10000; // Evitar explotar
        if (allPaths.size() >= MAX_PATHS) return;

        if (current == endNode) {
            List<Integer> path = new ArrayList<>(currentPath);
            path.add(current);
            allPaths.add(path);
            return;
        }

        if (currentPath.size() >= limitDist) return;

        visited.add(current);
        currentPath.add(current);

        for (int neigh : graph.get(current)) {
            if (!visited.contains(neigh)) {
                findAllPathsRecursiveOptimized(graph, neigh, endNode, visited, currentPath, allPaths, limitDist, size);
                  if (allPaths.size() >= MAX_PATHS) break;
            }
        }

        visited.remove(current);
        currentPath.remove(currentPath.size()-1);
    }

    private double calculatePathScore(List<Point> path, int size) {
        if (path.isEmpty()) return 0.0;
        return  path.size();
    }

    private double centralityFactor(Point p, int size) {
        double centerX = (size-1)/2.0;
        double centerY = (size-1)/2.0;
        double dist = Math.sqrt(Math.pow(p.x - centerX,2)+Math.pow(p.y - centerY,2));
        return 1.0/(dist+1);
    }

    private PlayerType getOpponentColor(PlayerType color) {
        return (color == PlayerType.PLAYER1)? PlayerType.PLAYER2 : PlayerType.PLAYER1;
    }

    private int colorToInt(PlayerType color) {
        return (color == PlayerType.PLAYER1) ? 1 : -1;
    }

    private List<Point> getLegalMoves(HexGameStatus s) {
        List<Point> moves = new ArrayList<>();
        int size = s.getSize();
        for (int i=0; i<size; i++) {
            for (int j=0; j<size; j++) {
                if (s.getPos(i,j) == 0) moves.add(new Point(i,j));
            }
        }
        return moves;
    }

    private HexGameStatus applyMove(HexGameStatus s, Point move) {
        HexGameStatus newState = new HexGameStatus(s);
        newState.placeStone(move);
        return newState;
    }

    private int[] getVirtualNodes(PlayerType color, int size) {
        int P1_START = size*size;
        int P1_END = size*size+1;
        int P2_START = size*size+2;
        int P2_END = size*size+3;
        if (color == PlayerType.PLAYER1) {
            return new int[]{P1_START, P1_END};
        } else {
            return new int[]{P2_START, P2_END};
        }
    }

    private List<List<Integer>> buildGraphForPaths(HexGameStatus s, PlayerType color) {
        int size = s.getSize();
        int totalNodes = size*size+4;
        List<List<Integer>> graph = new ArrayList<>(totalNodes);
        for (int i=0;i<totalNodes;i++) graph.add(new ArrayList<>());

        PlayerType opponent = getOpponentColor(color);
        int opponentVal = colorToInt(opponent);

        int P1_START = size*size;
        int P1_END = size*size+1;
        int P2_START = size*size+2;
        int P2_END = size*size+3;

        for (int x=0;x<size;x++){
            for(int y=0;y<size;y++){
                int cellVal = s.getPos(x,y);
                if (cellVal!=opponentVal) {
                    int u = x*size+y;
                    for (int[] d: HEX_DIRECTIONS) {
                        int nx=x+d[0], ny=y+d[1];
                        if (nx>=0 && nx<size && ny>=0 && ny<size) {
                            int neighVal=s.getPos(nx,ny);
                            if (neighVal!=opponentVal) {
                                int v=nx*size+ny;
                                graph.get(u).add(v);
                            }
                        }
                    }
                }
            }
        }

        if (color==PlayerType.PLAYER1) {
            for (int yy=0;yy<size;yy++){
                if(s.getPos(0,yy)!=opponentVal)
                    graph.get(P1_START).add(0*size+yy);
            }
            for (int yy=0;yy<size;yy++){
                if(s.getPos(size-1,yy)!=opponentVal)
                    graph.get((size-1)*size+yy).add(P1_END);
            }
        } else {
            for (int xx=0;xx<size;xx++){
                if(s.getPos(xx,0)!=opponentVal)
                    graph.get(P2_START).add(xx*size+0);
            }
            for (int xx=0;xx<size;xx++){
                if(s.getPos(xx,size-1)!=opponentVal)
                    graph.get(xx*size+(size-1)).add(P2_END);
            }
        }

        return graph;
    }

    private int calculateShortestDistanceDijkstra(List<List<Integer>> graph, int startNode, int endNode) {
        int n=graph.size();
        int[] dist=new int[n];
        Arrays.fill(dist,Integer.MAX_VALUE);
        dist[startNode]=0;
        PriorityQueue<int[]>pq=new PriorityQueue<>(Comparator.comparingInt(a->a[1]));
        pq.add(new int[]{startNode,0});
        while(!pq.isEmpty()){
            int[]cur=pq.poll();
            int u=cur[0],d=cur[1];
            if (d>dist[u]) continue;
            if (u==endNode)return d;
            for(int w:graph.get(u)){
                int nd=d+1;
                if(nd<dist[w]){
                    dist[w]=nd;
                    pq.add(new int[]{w,nd});
                }
            }
        }
        return Integer.MAX_VALUE;
    }

    private void filterGraph(List<List<Integer>> graph, int startNode, int endNode) {
        Set<Integer> fromStart=bfsReachable(graph,startNode);
        List<List<Integer>> rev=buildReverseGraph(graph);
        Set<Integer> fromEnd=bfsReachable(rev,endNode);
        Set<Integer>intersection=new HashSet<>(fromStart);
        intersection.retainAll(fromEnd);

        for(int u=0;u<graph.size();u++){
            if(!intersection.contains(u)){
                graph.get(u).clear();
            } else {
                graph.get(u).removeIf(v->!intersection.contains(v));
            }
        }
    }

    private Set<Integer> bfsReachable(List<List<Integer>> g,int start){
        Set<Integer>visited=new HashSet<>();
        Queue<Integer>q=new LinkedList<>();
        visited.add(start);
        q.add(start);
        while(!q.isEmpty()){
            int u=q.poll();
            for(int w:g.get(u)){
                if(!visited.contains(w)){
                    visited.add(w);
                    q.add(w);
                }
            }
        }
        return visited;
    }

    private List<List<Integer>> buildReverseGraph(List<List<Integer>> graph) {
        int n=graph.size();
        List<List<Integer>> rev=new ArrayList<>(n);
        for(int i=0;i<n;i++) rev.add(new ArrayList<>());
        for(int u=0;u<n;u++){
            for(int v:graph.get(u)){
                rev.get(v).add(u);
            }
        }
        return rev;
    }

    private List<List<Point>> convertPathsToPoints(List<List<Integer>> paths, int size) {
        List<List<Point>> result=new ArrayList<>();
        for(List<Integer> path:paths){
            List<Point> pPath=new ArrayList<>();
            for(int n:path){
                if(n<size*size){
                    int x=n/size;
                    int y=n%size;
                    pPath.add(new Point(x,y));
                }
            }
            if(!pPath.isEmpty())result.add(pPath);
        }
        return result;
    }

    // ---------------------------
    // Zobrist Hashing
    // ---------------------------
    private void initializeZobristTable(int size) {
        Random rand = new Random(123456789); // semilla fija o variable
        // Para cada casilla y para cada jugador posible
        // Tenemos 2 jugadores: Player1 y Player2
        // Indice jugador: 0 -> Player1, 1-> Player2
        zobristTable = new long[size*size][2];
        for (int i=0; i<size*size; i++) {
            for (int j=0; j<2; j++) {
                zobristTable[i][j] = rand.nextLong();
            }
        }
    }

    private long computeHash(HexGameStatus s) {
        long h = 0L;
        int size = s.getSize();
        for (int x=0; x<size; x++) {
            for (int y=0; y<size; y++) {
                int val = s.getPos(x,y);
                if (val != 0) {
                    int index = x*size+y;
                    int playerIndex = (val == 1)? 0 : 1;
                    h ^= zobristTable[index][playerIndex];
                }
            }
        }
        return h;
    }

    // Clase de entrada a la tabla de transposición
    private static class TranspositionEntry {
        public static final int EXACT = 0;
        public static final int LOWERBOUND = 1;
        public static final int UPPERBOUND = 2;

        int value;
        int depth;
        int flag;
        Point bestMove;

        TranspositionEntry(int value, int depth, int flag, Point bestMove) {
            this.value = value;
            this.depth = depth;
            this.flag = flag;
            this.bestMove = bestMove;
        }
    }
    
}
