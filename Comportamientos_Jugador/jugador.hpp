#ifndef COMPORTAMIENTOJUGADOR_H
#define COMPORTAMIENTOJUGADOR_H

#include "comportamientos/comportamiento.hpp"

#include <list>
#include <iostream>
#include <queue>

ostream& operator<<(ostream& out, const ubicacion& x);

struct stateN0 {
  ubicacion jugador;
  ubicacion sonambulo;
  bool operator== (const stateN0& x) const {
    if (jugador == x.jugador and sonambulo.f==x.sonambulo.f and sonambulo.c==x.sonambulo.c)
      return true;
    else 
      return false;
  };
};

struct nodeN0 {
  stateN0 st;
  list<Action> secuencia;
  bool operator==(const nodeN0& n) const {
    return (st==n.st);
  }
  bool operator<(const nodeN0& n) const {
    if (st.jugador.f < n.st.jugador.f)
      return true;
    else if(st.jugador.f == n.st.jugador.f and st.jugador.c<n.st.jugador.c)
      return true;
    else if (st.jugador.f == n.st.jugador.f and st.jugador.c==n.st.jugador.c and st.jugador.brujula < n.st.jugador.brujula)
      return true;
    else
      return false;
  }
};

struct nodeN1 {
  stateN0 st;
  list<Action> secuencia;
  bool operator==(const nodeN1& n) const {
    return (st==n.st);
  }
  bool operator<(const nodeN1& n) const {
    if (st.jugador.f < n.st.jugador.f)
      return true;
    else if(st.jugador.f == n.st.jugador.f and st.jugador.c<n.st.jugador.c)
      return true;
    else if (st.jugador.f == n.st.jugador.f and st.jugador.c==n.st.jugador.c and st.jugador.brujula < n.st.jugador.brujula)
      return true;
    else if (st.jugador.f == n.st.jugador.f and st.jugador.c==n.st.jugador.c and st.jugador.brujula == n.st.jugador.brujula and
      st.sonambulo.f < n.st.sonambulo.f)
      return true;
    else if (st.jugador.f == n.st.jugador.f and st.jugador.c==n.st.jugador.c and st.jugador.brujula == n.st.jugador.brujula and
      st.sonambulo.f == n.st.sonambulo.f and st.sonambulo.c < n.st.sonambulo.c)
      return true;
    else if (st.jugador.f == n.st.jugador.f and st.jugador.c==n.st.jugador.c and st.jugador.brujula == n.st.jugador.brujula and
      st.sonambulo.f == n.st.sonambulo.f and st.sonambulo.c == n.st.sonambulo.c and st.sonambulo.brujula<n.st.sonambulo.brujula)
      return true;
    else
      return false;
  }
};

struct nodeN2 {
  ubicacion jugador;
  int distancia;
  bool tieneBikini;
  bool tieneZapatillas;
  list<Action> secuencia;
  bool operator<(const nodeN2& n) const {
    return distancia>n.distancia;
  }
};

struct nodeN3 {
  stateN0 st;
  int g;
  int h;
  bool tieneBikini;
  bool tieneZapatillas;
  bool tieneBikiniSon;
  bool tieneZapatillasSon;
  list<Action> secuencia;
  bool operator<(const nodeN3& n) const {
    return (g+h)>(n.g+n.h);
  }
};

struct stateN4 {
  ubicacion jugador;
  ubicacion sonambulo;
  bool bienSituado;
  bool ubicandose;
  bool tieneBikini;
  bool tieneZapatillas;
  bool tieneBikiniSon;
  bool tieneZapatillasSon;
};

ostream& operator<<(ostream& out, const stateN0& x);

class ComportamientoJugador : public Comportamiento {
  public:
    ComportamientoJugador(unsigned int size) : Comportamiento(size) {
      last_action = actIDLE;
      current_stateN4.ubicandose = false;
      // Inicializamos las casillas de bordes:
      for (int i=0; i<3; i++) {
        for (int j=0; j<size; j++) {
          mapaResultado[i][j] = 'P';
          mapaResultado[j][i] = 'P';
        }
      }
      for (int i=size-1; i>size-4; i--) {
        for (int j=0; j<size; j++) {
          mapaResultado[i][j] = 'P';
          mapaResultado[j][i] = 'P';
        }
      }

      current_stateN4.bienSituado = current_stateN4.tieneBikini = current_stateN4.tieneZapatillas = current_stateN4.tieneBikiniSon = current_stateN4.tieneZapatillasSon = false;
      // Inicializar Variables de Estado
      // Inicializar Variables de Estado
      // Este vector de vectores de parejas nos dirá los desplazamientos necesarios para situar las casillas del terreno según nuestra orientación
    // * Primera coordenada: De 0 a 7 : indica la orientación
    // * Segunda coordenada: De 0 a 15 : indica la posición de la casilla del terreno visible
    // * El elemento first indica el desplazamiento en la fila y el last en la columna
    for (int i = 0; i < 8; ++i)
      casillasTerreno.push_back(vector<pair<int, int>>(16));

    // Norte
    vector<pair<int, int>> posiciones = {
        pair<int, int>(0, 0),
        pair<int, int>(-1, -1), pair<int, int>(-1, 0), pair<int, int>(-1, 1),
        pair<int, int>(-2, -2), pair<int, int>(-2, -1), pair<int, int>(-2, 0), pair<int, int>(-2, 1), pair<int, int>(-2, 2),
        pair<int, int>(-3, -3), pair<int, int>(-3, -2), pair<int, int>(-3, -1), pair<int, int>(-3, 0), pair<int, int>(-3, 1), pair<int, int>(-3, 2), pair<int, int>(-3, 3)};
    casillasTerreno[0] = posiciones;

    // Noreste
    posiciones = {
        pair<int, int>(0, 0),
        pair<int, int>(-1, 0), pair<int, int>(-1, 1), pair<int, int>(0, 1),
        pair<int, int>(-2, 0), pair<int, int>(-2, 1), pair<int, int>(-2, 2), pair<int, int>(-1, 2), pair<int, int>(0, 2),
        pair<int, int>(-3, 0), pair<int, int>(-3, 1), pair<int, int>(-3, 2), pair<int, int>(-3, 3), pair<int, int>(-2, 3), pair<int, int>(-1, 3), pair<int, int>(0, 3)};
    casillasTerreno[1] = posiciones;

    // Este
    posiciones = {
        pair<int, int>(0, 0),
        pair<int, int>(-1, 1), pair<int, int>(0, 1), pair<int, int>(1, 1),
        pair<int, int>(-2, 2), pair<int, int>(-1, 2), pair<int, int>(0, 2), pair<int, int>(1, 2), pair<int, int>(2, 2),
        pair<int, int>(-3, 3), pair<int, int>(-2, 3), pair<int, int>(-1, 3), pair<int, int>(0, 3), pair<int, int>(1, 3), pair<int, int>(2, 3), pair<int, int>(3, 3)};
    casillasTerreno[2] = posiciones;

    // Sureste
    posiciones = {
        pair<int, int>(0, 0),
        pair<int, int>(0, 1), pair<int, int>(1, 1), pair<int, int>(1, 0),
        pair<int, int>(0, 2), pair<int, int>(1, 2), pair<int, int>(2, 2), pair<int, int>(2, 1), pair<int, int>(2, 0),
        pair<int, int>(0, 3), pair<int, int>(1, 3), pair<int, int>(2, 3), pair<int, int>(3, 3), pair<int, int>(3, 2), pair<int, int>(3, 1), pair<int, int>(3, 0)};
    casillasTerreno[3] = posiciones;

    // Sur
    posiciones = {
        pair<int, int>(0, 0),
        pair<int, int>(1, 1), pair<int, int>(1, 0), pair<int, int>(1, -1),
        pair<int, int>(2, 2), pair<int, int>(2, 1), pair<int, int>(2, 0), pair<int, int>(2, -1), pair<int, int>(2, -2),
        pair<int, int>(3, 3), pair<int, int>(3, 2), pair<int, int>(3, 1), pair<int, int>(3, 0), pair<int, int>(3, -1), pair<int, int>(3, -2), pair<int, int>(3, -3)};
    casillasTerreno[4] = posiciones;

    // Sin hacer: Suroeste
    posiciones = {
        pair<int, int>(0, 0),
        pair<int, int>(1, 0), pair<int, int>(1, -1), pair<int, int>(0, -1),
        pair<int, int>(2, 0), pair<int, int>(2, -1), pair<int, int>(2, -2), pair<int, int>(1, -2), pair<int, int>(0, -2),
        pair<int, int>(3, 0), pair<int, int>(3, -1), pair<int, int>(3, -2), pair<int, int>(3, -3), pair<int, int>(2, -3), pair<int, int>(1, -3), pair<int, int>(0, -3)};
    casillasTerreno[5] = posiciones;

    // Oeste
    posiciones = {
        pair<int, int>(0, 0),
        pair<int, int>(1, -1), pair<int, int>(0, -1), pair<int, int>(-1, -1),
        pair<int, int>(2, -2), pair<int, int>(1, -2), pair<int, int>(0, -2), pair<int, int>(-1, -2), pair<int, int>(-2, -2),
        pair<int, int>(3, -3), pair<int, int>(2, -3), pair<int, int>(1, -3), pair<int, int>(0, -3), pair<int, int>(-1, -3), pair<int, int>(-2, -3), pair<int, int>(-3, -3)};
    casillasTerreno[6] = posiciones;

    // Sin hacer : Noroeste
    posiciones = {
        pair<int, int>(0, 0),
        pair<int, int>(0, -1), pair<int, int>(-1, -1), pair<int, int>(-1, 0),
        pair<int, int>(0, -2), pair<int, int>(-1, -2), pair<int, int>(-2, -2), pair<int, int>(-2, -1), pair<int, int>(-2, 0),
        pair<int, int>(0, -3), pair<int, int>(-1, -3), pair<int, int>(-2, -3), pair<int, int>(-3, -3), pair<int, int>(-3, -2), pair<int, int>(-3, -1), pair<int, int>(-3, 0)};
    casillasTerreno[7] = posiciones;
    hayPlan = false;
    }
    ComportamientoJugador(std::vector< std::vector< unsigned char> > mapaR) : Comportamiento(mapaR) {
      // Inicializar Variables de Estado
      // Inicializar Variables de Estado
      // Este vector de vectores de parejas nos dirá los desplazamientos necesarios para situar las casillas del terreno según nuestra orientación
    // * Primera coordenada: De 0 a 7 : indica la orientación
    // * Segunda coordenada: De 0 a 15 : indica la posición de la casilla del terreno visible
    // * El elemento first indica el desplazamiento en la fila y el last en la columna
    for (int i = 0; i < 8; ++i)
      casillasTerreno.push_back(vector<pair<int, int>>(16));

    // Norte
    vector<pair<int, int>> posiciones = {
        pair<int, int>(0, 0),
        pair<int, int>(-1, -1), pair<int, int>(-1, 0), pair<int, int>(-1, 1),
        pair<int, int>(-2, -2), pair<int, int>(-2, -1), pair<int, int>(-2, 0), pair<int, int>(-2, 1), pair<int, int>(-2, 2),
        pair<int, int>(-3, -3), pair<int, int>(-3, -2), pair<int, int>(-3, -1), pair<int, int>(-3, 0), pair<int, int>(-3, 1), pair<int, int>(-3, 2), pair<int, int>(-3, 3)};
    casillasTerreno[0] = posiciones;

    // Noreste
    posiciones = {
        pair<int, int>(0, 0),
        pair<int, int>(-1, 0), pair<int, int>(-1, 1), pair<int, int>(0, 1),
        pair<int, int>(-2, 0), pair<int, int>(-2, 1), pair<int, int>(-2, 2), pair<int, int>(-1, 2), pair<int, int>(0, 2),
        pair<int, int>(-3, 0), pair<int, int>(-3, 1), pair<int, int>(-3, 2), pair<int, int>(-3, 3), pair<int, int>(-2, 3), pair<int, int>(-1, 3), pair<int, int>(0, 3)};
    casillasTerreno[1] = posiciones;

    // Este
    posiciones = {
        pair<int, int>(0, 0),
        pair<int, int>(-1, 1), pair<int, int>(0, 1), pair<int, int>(1, 1),
        pair<int, int>(-2, 2), pair<int, int>(-1, 2), pair<int, int>(0, 2), pair<int, int>(1, 2), pair<int, int>(2, 2),
        pair<int, int>(-3, 3), pair<int, int>(-2, 3), pair<int, int>(-1, 3), pair<int, int>(0, 3), pair<int, int>(1, 3), pair<int, int>(2, 3), pair<int, int>(3, 3)};
    casillasTerreno[2] = posiciones;

    // Sureste
    posiciones = {
        pair<int, int>(0, 0),
        pair<int, int>(0, 1), pair<int, int>(1, 1), pair<int, int>(1, 0),
        pair<int, int>(0, 2), pair<int, int>(1, 2), pair<int, int>(2, 2), pair<int, int>(2, 1), pair<int, int>(2, 0),
        pair<int, int>(0, 3), pair<int, int>(1, 3), pair<int, int>(2, 3), pair<int, int>(3, 3), pair<int, int>(3, 2), pair<int, int>(3, 1), pair<int, int>(3, 0)};
    casillasTerreno[3] = posiciones;

    // Sur
    posiciones = {
        pair<int, int>(0, 0),
        pair<int, int>(1, 1), pair<int, int>(1, 0), pair<int, int>(1, -1),
        pair<int, int>(2, 2), pair<int, int>(2, 1), pair<int, int>(2, 0), pair<int, int>(2, -1), pair<int, int>(2, -2),
        pair<int, int>(3, 3), pair<int, int>(3, 2), pair<int, int>(3, 1), pair<int, int>(3, 0), pair<int, int>(3, -1), pair<int, int>(3, -2), pair<int, int>(3, -3)};
    casillasTerreno[4] = posiciones;

    // Sin hacer: Suroeste
    posiciones = {
        pair<int, int>(0, 0),
        pair<int, int>(1, 0), pair<int, int>(1, -1), pair<int, int>(0, -1),
        pair<int, int>(2, 0), pair<int, int>(2, -1), pair<int, int>(2, -2), pair<int, int>(1, -2), pair<int, int>(0, -2),
        pair<int, int>(3, 0), pair<int, int>(3, -1), pair<int, int>(3, -2), pair<int, int>(3, -3), pair<int, int>(2, -3), pair<int, int>(1, -3), pair<int, int>(0, -3)};
    casillasTerreno[5] = posiciones;

    // Oeste
    posiciones = {
        pair<int, int>(0, 0),
        pair<int, int>(1, -1), pair<int, int>(0, -1), pair<int, int>(-1, -1),
        pair<int, int>(2, -2), pair<int, int>(1, -2), pair<int, int>(0, -2), pair<int, int>(-1, -2), pair<int, int>(-2, -2),
        pair<int, int>(3, -3), pair<int, int>(2, -3), pair<int, int>(1, -3), pair<int, int>(0, -3), pair<int, int>(-1, -3), pair<int, int>(-2, -3), pair<int, int>(-3, -3)};
    casillasTerreno[6] = posiciones;

    // Sin hacer : Noroeste
    posiciones = {
        pair<int, int>(0, 0),
        pair<int, int>(0, -1), pair<int, int>(-1, -1), pair<int, int>(-1, 0),
        pair<int, int>(0, -2), pair<int, int>(-1, -2), pair<int, int>(-2, -2), pair<int, int>(-2, -1), pair<int, int>(-2, 0),
        pair<int, int>(0, -3), pair<int, int>(-1, -3), pair<int, int>(-2, -3), pair<int, int>(-3, -3), pair<int, int>(-3, -2), pair<int, int>(-3, -1), pair<int, int>(-3, 0)};
    casillasTerreno[7] = posiciones;
    hayPlan = false;
    }
    ComportamientoJugador(const ComportamientoJugador & comport) : Comportamiento(comport){}
    ~ComportamientoJugador(){}

    Action think(Sensores sensores);
    int interact(Action accion, int valor);

    // NIVEL 4
    void Nivel4(Sensores sensores);

    // DEBUG
    void VisualizaPlan(const stateN0& st, const list<Action>& plan);

  private:
    // Declarar Variables de Estado
    list<Action> plan;
    bool hayPlan;
    stateN0 current_state;
    stateN4 current_stateN4;
    Action last_action;
    ubicacion goal;
    // Vector para saber de forma intuitiva cuáles son las casillas del mapa (fila, columna) que tiene el agente en su visión
    vector<vector<pair<int,int>>> casillasTerreno;
};

// Nivel 0
list<Action> AnchuraSoloJugador(const stateN0& inicio, const ubicacion& final, const vector<vector<unsigned char>>& mapa);
bool Find(const list<stateN0>& lista, const stateN0& obj);
bool Find(const list<nodeN0>& lista, const stateN0& obj);

// Nivel 1
list<Action> AnchuraSonambulo(const stateN0& inicio, const ubicacion& final, const vector<vector<unsigned char>>& mapa, const vector<vector<pair<int,int>>>& casillasTerreno);
bool EsVisible(const stateN0& st, const vector<vector<pair<int,int>>>& casillasTerreno);

// Nivel 2
list<Action> DijkstraSoloJugador(const stateN0& inicio, const ubicacion& final, const vector<vector<unsigned char>>& mapa);
ubicacion apply(Action action, const ubicacion& current_state, const vector<vector<unsigned char>>& mapa, const pair<int,int>& sonambulo);
int Distancia(const nodeN2 &origen, Action accion, const vector<vector<unsigned char>> &mapa);
bool Find(const list<nodeN2>& lista, const nodeN2& obj);

// Nivel 3
list<Action> AStar(const stateN0& inicio, const ubicacion& final, const vector<vector<unsigned char>>& mapa, const vector<vector<pair<int,int>>>& casillasTerreno);
bool Find(const list<nodeN3>& lista, const nodeN3& obj);
int Distancia(const nodeN3 &origen, Action accion, const vector<vector<unsigned char>> &mapa, bool sonambulo);
int DistanciaManhattan(const ubicacion &origen, const ubicacion &destino);
int DistanciaChebyshev(const ubicacion& origen, const ubicacion& destino);
int DistanciaChebyshevMejorada(const nodeN3 &origen, const ubicacion &destino, const unsigned char &casillaOrigenSon, const unsigned char &casillaOrigenJug);
bool cmpN3(nodeN3 a, nodeN3 n);
void print_queue(priority_queue<nodeN3> q, vector<vector<unsigned char>>& mapaConPlan);

// Nivel 4
void rellenaMapa(const vector<unsigned char> &terreno, vector<vector<unsigned char>> &mapa, const stateN4 &st, const vector<vector<pair<int, int>>> &casillasTerreno, const Sensores &sensores);
stateN4 UpdateState(const stateN4& st, const Action accion);
void rellenaMapa(const vector<unsigned char> &terreno, vector<vector<unsigned char>> &mapa, const ubicacion &st, const vector<vector<pair<int, int>>> &casillasTerreno);

bool CasillaTransitable(const ubicacion& x, const vector<vector<unsigned char>>& mapa);
ubicacion NextCasilla(const ubicacion& pos);
stateN0 apply(Action action, const stateN0& current_state, const vector<vector<unsigned char>>& mapa);

void AnularMatriz(vector<vector<unsigned char>>& matriz);

#endif
