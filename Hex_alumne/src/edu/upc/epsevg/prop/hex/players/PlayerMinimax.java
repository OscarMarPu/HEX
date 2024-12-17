package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.*;
import java.awt.Point;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * Jugador que utiliza el algoritmo Minimax con poda alfa-beta y tablas de transposición.
 * Ajustes:
 * - Se ordenan los movimientos SOLO en la raíz para ayudar a la poda.
 * - En niveles profundos NO se ordenan los movimientos, reduciendo el coste.
 * - Se mantiene la perspectiva coherente del jugador actual.
 */
public class PlayerMinimax implements IPlayer, IAuto {

    private int profmax;            // Profundidad máxima
    private String name;            // Nombre del jugador
    private PlayerType myColor;     // Nuestro color (PLAYER1 o PLAYER2)
    private long nodesvisitats;     // Nodos visitados
    private long nodesestalviats;   // Nodos podados
    private static final int MAXIM = 100000000;
    private HexHeuristica heuristica;
    private HashMap<String, TranspositionEntry> transpositionTable;

    /**
     * Constructor que inicializa el jugador con la profundidad máxima.
     * @param name Nombre del jugador
     * @param prof Profundidad máxima de exploración
     */
    public PlayerMinimax(String name, int prof) {
        this.name = name;
        this.profmax = prof;
        this.heuristica = new HexHeuristica();
        this.transpositionTable = new HashMap<>();
    }

    @Override
    public PlayerMove move(HexGameStatus s) {
        nodesvisitats = 0;
        nodesestalviats = 0;
        myColor = s.getCurrentPlayer();

        int alpha = -MAXIM;
        int beta = MAXIM;
        Point mejorMovimiento = null;
        int mejorValor = Integer.MIN_VALUE;

        // Ordenamos movimientos solo en la raíz para mejorar la poda.
        List<Point> movimientosLegales = ordenarMovimientosRaiz(s);

        for (Point movimiento : movimientosLegales) {
            HexGameStatus nuevoEstado = applyMove(s, movimiento);
            int valor = minValor(nuevoEstado, alpha, beta, profmax - 1);
            if (valor > mejorValor) {
                mejorValor = valor;
                mejorMovimiento = movimiento;
            }
            alpha = Math.max(alpha, valor);
            if (beta <= alpha) {
                nodesestalviats++;
                break; // poda alfa-beta
            }
        }

        return new PlayerMove(mejorMovimiento, nodesvisitats, profmax, SearchType.MINIMAX);
    }

    @Override
    public String getName() {
        return "Minimax(" + name + ")";
    }

    @Override
    public void timeout() {
        System.out.println("Timeout occurred.");
    }

    private int maxValor(HexGameStatus s, int alpha, int beta, int profundidad) {
        String clave = generarClaveEstado(s);

        // Tabla de transposición
        if (transpositionTable.containsKey(clave)) {
            TranspositionEntry entrada = transpositionTable.get(clave);
            // Usamos el resultado si la profundidad de la entrada es >= a la actual
            if (entrada.profundidad >= profundidad && entrada.alpha <= alpha && entrada.beta >= beta) {
                return entrada.valor;
            }
        }

        if (profundidad == 0 || s.isGameOver()) {
            return evaluarEstado(s);
        }

        int valor = Integer.MIN_VALUE;
        List<Point> movimientos = getLegalMoves(s); // Sin ordenar en niveles profundos

        for (Point movimiento : movimientos) {
            HexGameStatus nuevoEstado = applyMove(s, movimiento);
            valor = Math.max(valor, minValor(nuevoEstado, alpha, beta, profundidad - 1));
            if (valor >= beta) {
                nodesestalviats++;
                transpositionTable.put(clave, new TranspositionEntry(valor, profundidad, alpha, beta));
                return valor;
            }
            alpha = Math.max(alpha, valor);
        }

        transpositionTable.put(clave, new TranspositionEntry(valor, profundidad, alpha, beta));
        return valor;
    }

    private int minValor(HexGameStatus s, int alpha, int beta, int profundidad) {
        String clave = generarClaveEstado(s);

        // Tabla de transposición
        if (transpositionTable.containsKey(clave)) {
            TranspositionEntry entrada = transpositionTable.get(clave);
            if (entrada.profundidad >= profundidad && entrada.alpha <= alpha && entrada.beta >= beta) {
                return entrada.valor;
            }
        }

        if (profundidad == 0 || s.isGameOver()) {
            return evaluarEstado(s);
        }

        int valor = Integer.MAX_VALUE;
        List<Point> movimientos = getLegalMoves(s); // Sin ordenar en niveles profundos

        for (Point movimiento : movimientos) {
            HexGameStatus nuevoEstado = applyMove(s, movimiento);
            valor = Math.min(valor, maxValor(nuevoEstado, alpha, beta, profundidad - 1));
            if (valor <= alpha) {
                nodesestalviats++;
                transpositionTable.put(clave, new TranspositionEntry(valor, profundidad, alpha, beta));
                return valor;
            }
            beta = Math.min(beta, valor);
        }

        transpositionTable.put(clave, new TranspositionEntry(valor, profundidad, alpha, beta));
        return valor;
    }

    /**
     * Evaluación del estado desde la perspectiva de 'myColor'.
     * Si el juego ha terminado, el valor es extremo a favor o en contra.
     * En otros casos, se usa la heurística.
     */
    private int evaluarEstado(HexGameStatus s) {
        nodesvisitats++;
        if (s.isGameOver()) {
            PlayerType ganador = s.GetWinner();
            return (ganador == myColor) ? MAXIM : -MAXIM;
        }
        // Profundidad 2 es un número arbitrario: la heurística interna ya adapta su complejidad
        return heuristica.evaluarEstado(s, 2);
    }

    /**
     * Ordena los movimientos solo al nivel raíz, evaluando cada movimiento una sola vez.
     * Esto ayudará a la poda alfa-beta inicial, sin incurrir en costes adicionales en niveles profundos.
     */
    private List<Point> ordenarMovimientosRaiz(HexGameStatus s) {
        List<Point> movimientos = getLegalMoves(s);

        movimientos.sort((m1, m2) -> {
            HexGameStatus estado1 = applyMove(s, m1);
            HexGameStatus estado2 = applyMove(s, m2);
            int valor1 = heuristica.evaluarEstado(estado1, 1);
            int valor2 = heuristica.evaluarEstado(estado2, 1);
            // Al inicio, somos el jugador que maximiza (myColor).
            return Integer.compare(valor2, valor1);
        });

        return movimientos;
    }

    private HexGameStatus applyMove(HexGameStatus s, Point movimiento) {
        HexGameStatus nuevoEstado = new HexGameStatus(s);
        nuevoEstado.placeStone(movimiento);
        return nuevoEstado;
    }

    private List<Point> getLegalMoves(HexGameStatus s) {
        List<Point> movimientos = new ArrayList<>();
        int size = s.getSize();
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                if (s.getPos(i, j) == 0) {
                    movimientos.add(new Point(i, j));
                }
            }
        }
        return movimientos;
    }

    private String generarClaveEstado(HexGameStatus s) {
        StringBuilder clave = new StringBuilder();
        int size = s.getSize();
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                clave.append(s.getPos(i, j));
            }
        }
        return clave.toString();
    }

    private static class TranspositionEntry {
        int valor;
        int profundidad;
        int alpha;
        int beta;

        public TranspositionEntry(int valor, int profundidad, int alpha, int beta) {
            this.valor = valor;
            this.profundidad = profundidad;
            this.alpha = alpha;
            this.beta = beta;
        }
    }
}
