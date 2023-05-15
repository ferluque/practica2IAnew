#include "../Comportamientos_Jugador/jugador.hpp"
#include "motorlib/util.h"

#include <iostream>
#include <cmath>
#include <set>
#include <stack>

#include <time.h>

// Este es el método principal que se piden en la practica.
// Tiene como entrada la información de los sensores y devuelve la acción a realizar.
// Para ver los distintos sensores mirar fichero "comportamiento.hpp"
Action ComportamientoJugador::think(Sensores sensores)
{
	Action accion = actIDLE;
	if (sensores.nivel == 4)
	{
		current_stateN4 = UpdateState(current_stateN4, last_action);
		// cout << "Situación actual: " << endl
		// 	 << current_stateN4.jugador << current_stateN4.sonambulo << endl;
	}
	if (!hayPlan)
	{
		cout << "Calculando un nuevo plan" << endl;
		current_state.jugador.f = sensores.posF;
		current_state.jugador.c = sensores.posC;
		current_state.jugador.brujula = static_cast<Orientacion>(sensores.sentido % 8);
		current_state.sonambulo.f = sensores.SONposF;
		current_state.sonambulo.c = sensores.SONposC;
		current_state.sonambulo.brujula = static_cast<Orientacion>(sensores.SONsentido % 8);
		goal.f = sensores.destinoF;
		goal.c = sensores.destinoC;

		switch (sensores.nivel)
		{
		case 0:
			plan = AnchuraSoloJugador(current_state, goal, mapaResultado);
			break;
		case 1:
			plan = AnchuraSonambulo(current_state, goal, mapaResultado, casillasTerreno);
			break;
		case 2:
			plan = DijkstraSoloJugador(current_state, goal, mapaResultado);
			break;
		case 3:
			plan = AStar(current_state, goal, mapaResultado, casillasTerreno);
			break;
		case 4:
			Nivel4(sensores);
			break;
		}
		if (plan.size() > 0)
		{
			stateN0 st;
			if (sensores.nivel == 4)
			{
				// cout << "Plan restante: ";
				// for (auto it=plan.begin(); it!=plan.end(); ++it)
				// 	cout << *it << ", ";
				// cout << endl;
				st.jugador = current_stateN4.jugador;
				st.sonambulo = current_stateN4.sonambulo;
			}
			else
				st = current_state;
			VisualizaPlan(st, plan);
			hayPlan = true;
		}
	}
	if (hayPlan and plan.size() > 0)
	{
		accion = plan.front();
		plan.pop_front();

		if (sensores.nivel == 4 and !current_stateN4.ubicandose and current_stateN4.bienSituado)
		{
			rellenaMapa(sensores.terreno, mapaResultado, current_stateN4.jugador, casillasTerreno);
			if (sensores.reset)
			{
				list<Action> empty;
				swap(plan, empty);
				accion = actWHEREIS;
				current_stateN4.ubicandose = true;
			}
			if (sensores.colision)
			{
				list<Action> empty;
				swap(plan, empty);
				accion = actFORWARD;
				current_stateN4.bienSituado = false;
			}
			if (accion == actFORWARD and (sensores.terreno[2] == 'M' or sensores.terreno[2] == 'P' or sensores.superficie[2] != '_'))
			{
				cout << "Vacía plan" << endl;
				list<Action> empty;
				swap(plan, empty);
				accion = ((rand() % 2) == 0) ? actTURN_L : actTURN_R;
			}
			// Paramos también cuando el sonámbulo vaya a chocar o tirarse por unprecipicio
			if (accion == actSON_FORWARD)
			{
				stateN4 avanzaSon = UpdateState(current_stateN4, actSON_FORWARD);
				if (mapaResultado[avanzaSon.sonambulo.f][avanzaSon.sonambulo.c] == 'M' or mapaResultado[avanzaSon.sonambulo.f][avanzaSon.sonambulo.c] == 'P')
				{
					accion = ((rand() % 2) == 0) ? actSON_TURN_SL : actSON_TURN_SR;
					cout << "Vacía plan" << endl;
					list<Action> empty;
					swap(plan, empty);
				}
			}
		}
	}
	if (plan.size() == 0)
	{
		cout << "Se completó el plan" << endl;
		hayPlan = false;
	}
	last_action = accion;
	return accion;
}

// ==============================================
// NIVEL 0 ======================================
// ==============================================
list<Action> AnchuraSoloJugador(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa)
{
	nodeN0 current_node;
	list<nodeN0> frontier;
	set<nodeN0> explored;
	list<Action> plan;
	current_node.st = inicio;
	frontier.push_back(current_node);
	bool SolutionFound = (current_node.st.jugador.f == final.f and current_node.st.jugador.c == final.c);
	while (!frontier.empty() and !SolutionFound)
	{
		frontier.pop_front();
		explored.insert(current_node);
		// Hijo actFORWARD
		nodeN0 childForward = current_node;
		childForward.st = apply(actFORWARD, current_node.st, mapa);
		// Comprobamos que actFORWARD sea la solución
		if (childForward.st.jugador.f == final.f and childForward.st.jugador.c == final.c)
		{
			childForward.secuencia.push_back(actFORWARD);
			current_node = childForward;
			SolutionFound = true;
		}
		else if (explored.find(childForward) == explored.end())
		{
			childForward.secuencia.push_back(actFORWARD);
			frontier.push_back(childForward);
		}
		if (!SolutionFound)
		{
			// Hijo turnL
			nodeN0 childTurnl = current_node;
			childTurnl.st = apply(actTURN_L, current_node.st, mapa);
			if (explored.find(childTurnl) == explored.end())
			{
				childTurnl.secuencia.push_back(actTURN_L);
				frontier.push_back(childTurnl);
			}
			// Hijo turnR
			nodeN0 childTurnr = current_node;
			childTurnr.st = apply(actTURN_R, current_node.st, mapa);
			if (explored.find(childTurnr) == explored.end())
			{
				childTurnr.secuencia.push_back(actTURN_R);
				frontier.push_back(childTurnr);
			}
		}
		// Evitamos expandir nodos repetidos
		if (!SolutionFound and !frontier.empty())
		{
			current_node = frontier.front();
			while (!frontier.empty() and explored.find(current_node) != explored.end())
			{
				frontier.pop_front();
				if (!frontier.empty())
					current_node = frontier.front();
			}
		}
	}
	if (SolutionFound)
		plan = current_node.secuencia;
	return plan;
}

// ==============================================
// NIVEL 1 ======================================
// ==============================================
list<Action> AnchuraSonambulo(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa,
							  const vector<vector<pair<int, int>>> &casillasTerreno)
{
	nodeN1 current_node;
	list<nodeN1> frontier;
	set<nodeN1> explored;
	list<Action> plan;
	current_node.st = inicio;
	frontier.push_back(current_node);
	bool SolutionFound = (current_node.st.sonambulo.f == final.f and current_node.st.sonambulo.c == final.c);
	bool print = true;
	int limit = -1;
	int cont = 0;
	while (!frontier.empty() and !SolutionFound)
	{
		frontier.pop_front();
		explored.insert(current_node);
		// Generamos el hijo actSON_FORWARD y comprobamos si es solución antes de meterlo en cerrados
		if (EsVisible(current_node.st, casillasTerreno))
		{
			nodeN1 childSForward = current_node;
			childSForward.st = apply(actSON_FORWARD, current_node.st, mapa);
			if (childSForward.st.sonambulo.f == final.f and childSForward.st.sonambulo.c == final.c)
			{
				childSForward.secuencia.push_back(actSON_FORWARD);
				current_node = childSForward;
				SolutionFound = true;
			}
			// En caso de que apply devuelva el mismo nodo (no se ha podido aplicar la acción)
			// evitamos una búsqueda en explored
			if (!SolutionFound and (current_node < childSForward or childSForward < current_node))
			{
				if (explored.find(childSForward) == explored.end())
				{
					childSForward.secuencia.push_back(actSON_FORWARD);
					frontier.push_back(childSForward);
				}
			}
		}
		if (!SolutionFound)
		{
			// Hijo actFORWARD
			nodeN1 childForward = current_node;
			childForward.st = apply(actFORWARD, current_node.st, mapa);
			// En caso de que apply devuelva el mismo nodo (no se ha podido aplicar la acción)
			// evitamos una búsqueda en explored
			if (current_node < childForward or childForward < current_node)
			{
				if (explored.find(childForward) == explored.end())
				{
					childForward.secuencia.push_back(actFORWARD);
					frontier.push_back(childForward);
				}
			}
			// Hijo actTURN_L
			nodeN1 childTurnl = current_node;
			childTurnl.st = apply(actTURN_L, current_node.st, mapa);
			// En caso de que apply devuelva el mismo nodo (no se ha podido aplicar la acción)
			// evitamos una búsqueda en explored
			if (current_node < childTurnl or childTurnl < current_node)
			{
				if (explored.find(childTurnl) == explored.end())
				{
					childTurnl.secuencia.push_back(actTURN_L);
					frontier.push_back(childTurnl);
				}
			}
			nodeN1 childTurnr = current_node;
			childTurnr.st = apply(actTURN_R, current_node.st, mapa);
			// En caso de que apply devuelva el mismo nodo (no se ha podido aplicar la acción)
			// evitamos una búsqueda en explored
			if (current_node < childTurnr or childTurnr < current_node)
			{
				if (explored.find(childTurnr) == explored.end())
				{
					childTurnr.secuencia.push_back(actTURN_R);
					frontier.push_back(childTurnr);
				}
			}
			if (EsVisible(current_node.st, casillasTerreno))
			{
				// Hijo actSON_TURN_SR
				nodeN1 childTurnSR = current_node;
				childTurnSR.st = apply(actSON_TURN_SR, current_node.st, mapa);
				if (current_node < childTurnSR or childTurnSR < current_node)
				{
					if (explored.find(childTurnSR) == explored.end())
					{
						// cout << "Mete Hijo actSON_TURN_SR"<<endl;
						childTurnSR.secuencia.push_back(actSON_TURN_SR);
						frontier.push_back(childTurnSR);
					}
				}
				// Hijo actSON_TURN_SL
				nodeN1 childTurnSL = current_node;
				childTurnSL.st = apply(actSON_TURN_SL, current_node.st, mapa);
				if (current_node < childTurnSR or childTurnSR < current_node)
				{
					if (explored.find(childTurnSL) == explored.end())
					{
						// cout << "Mete Hijo actSON_TURN_SL"<<endl;
						childTurnSL.secuencia.push_back(actSON_TURN_SL);
						frontier.push_back(childTurnSL);
					}
				}
			}
		}
		// Evitamos expandir nodos repetidos
		if (!SolutionFound and !frontier.empty())
		{
			current_node = frontier.front();
			while (!frontier.empty() and explored.find(current_node) != explored.end())
			{
				frontier.pop_front();
				if (!frontier.empty())
					current_node = frontier.front();
			}
		}
	}
	if (SolutionFound)
		plan = current_node.secuencia;
	return plan;
}

// Comprobar si el sonámbulo está en la visión del jugador
bool EsVisible(const stateN0 &st, const vector<vector<pair<int, int>>> &casillasTerreno)
{
	vector<pair<int, int>> v(16);
	vector<pair<int, int>> casillas(casillasTerreno[static_cast<int>(st.jugador.brujula)]);

	bool found = false;
	for (int i = 0; i < 16 and !found; i++)
	{
		int f = st.jugador.f + casillas[i].first, c = st.jugador.c + casillas[i].second;
		if (f == st.sonambulo.f and c == st.sonambulo.c)
			found = true;
	}
	return found;
}

// ==============================================
// NIVEL 2 ======================================
// ==============================================

// Para intentar reducir la memoria ocupada, en este nivel el nodo no almacena la ubicación del sonámbulo para cada nodo,
// ya que el sonámbulo estará siempre en la misma posición. Para esto, guardamos la posición al inicio y usamos esa siempre 
ubicacion apply(Action action, const ubicacion &current_state, const vector<vector<unsigned char>> &mapa, const pair<int, int> &sonambulo)
{
	ubicacion stResult = current_state;
	ubicacion sigUbicacion;
	switch (action)
	{
	case actFORWARD:
		sigUbicacion = NextCasilla(current_state);
		if (CasillaTransitable(sigUbicacion, mapa) and
			!(sigUbicacion.f == sonambulo.first and sigUbicacion.c == sonambulo.second))
		{
			stResult = sigUbicacion;
		}
		break;
	case actTURN_L:
		stResult.brujula = static_cast<Orientacion>((stResult.brujula + 6) % 8);
		break;
		break;
	case actTURN_R:
		stResult.brujula = static_cast<Orientacion>((stResult.brujula + 2) % 8);
		break;
	}
	return stResult;
}

bool cmpN2(nodeN2 a, nodeN2 n)
{
	ubicacion st = a.jugador;
	if (st.f < n.jugador.f)
		return true;
	else if (st.f == n.jugador.f and st.c < n.jugador.c)
		return true;
	else if (st.f == n.jugador.f and st.c == n.jugador.c and st.brujula < n.jugador.brujula)
		return true;
	else if (st.f == n.jugador.f and st.c == n.jugador.c and st.brujula == n.jugador.brujula and
			 a.tieneBikini < n.tieneBikini)
		return true;
	else if (st.f == n.jugador.f and st.c == n.jugador.c and st.brujula == n.jugador.brujula and
			 a.tieneBikini == n.tieneBikini and a.tieneZapatillas < n.tieneZapatillas)
		return true;
	else
		return false;
}

list<Action> DijkstraSoloJugador(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa)
{
	list<Action> plan;
	pair<int, int> sonambulo(inicio.sonambulo.f, inicio.sonambulo.c);
	// Creamos esta matriz de 4 dimensiones para almacenar los nodos cerrados, en lugar
	// de usar un set
	// i: fila, j: columna, k: orientacion, l: 0 nada, 1 zapatillas, 2 bikini
	vector<vector<vector<vector<bool>>>> cerrados(mapa.size());
	for (int i = 0; i < mapa.size(); ++i)
	{
		cerrados[i] = vector<vector<vector<bool>>>(mapa.size());
		for (int j = 0; j < mapa.size(); j++)
		{
			cerrados[i][j] = vector<vector<bool>>(8);
			for (int k = 0; k < 8; ++k)
			{
				cerrados[i][j][k] = vector<bool>(3);
				for (int l = 0; l < 3; ++l)
					cerrados[i][j][k][l] = false;
			}
		}
	}
	priority_queue<nodeN2> abiertos;
	nodeN2 actual;
	actual.distancia = 0;
	actual.jugador = inicio.jugador;
	actual.tieneBikini = false;
	actual.tieneZapatillas = false;
	bool fin = (actual.jugador.f == final.f and actual.jugador.c == final.c);
	abiertos.push(actual);
	int cont = 0;
	int limit = -1;
	int contBusquedas = 0;
	clock_t iniciot;
	float tiempoFind = 0.0, tiempoPushCerrados = 0.0, tiempoPushAbiertos = 0.0;
	while (!fin)
	{
		actual = abiertos.top();
		// Si pasamos por un bikini lo cogemos y soltamos las zapatillas
		if (mapa[actual.jugador.f][actual.jugador.c] == 'K')
		{
			actual.tieneBikini = true;
			actual.tieneZapatillas = false;
		}
		// Y viceversa
		if (mapa[actual.jugador.f][actual.jugador.c] == 'D')
		{
			actual.tieneZapatillas = true;
			actual.tieneBikini = false;
		}
		int l = 0;
		if (actual.tieneZapatillas)
			l = 1;
		if (actual.tieneBikini)
			l = 2;

		abiertos.pop();
		// Anotamos el nodo actual como cerrado
		cerrados[actual.jugador.f][actual.jugador.c][actual.jugador.brujula][l] = true;

		// Comprobamos si es el objetio
		fin = (actual.jugador.f == final.f and actual.jugador.c == final.c);
		if (fin)
		{
			plan = actual.secuencia;
		}
		else
		{
			// Hijo actFORWARD
			nodeN2 childForward = actual;
			childForward.jugador = apply(actFORWARD, actual.jugador, mapa, sonambulo);
			// MODIFICACIÓN PARA EFICIENCIA
			// NO BUSCAR EN CERRADOS SI NO HA CAMBIADO, EVITA UNA BÚSQUEDA PARA AQUELLAS EXPANSIONES IMPOSIBLES
			if (cmpN2(actual, childForward) or cmpN2(childForward, actual))
			{
				if (!cerrados[childForward.jugador.f][childForward.jugador.c][childForward.jugador.brujula][l])
				{
					childForward.distancia += Distancia(actual, actFORWARD, mapa);
					childForward.secuencia.push_back(actFORWARD);
					abiertos.push(childForward);
				}
			}
			// Hijo actTURNL
			nodeN2 childTurnL = actual;
			childTurnL.jugador = apply(actTURN_L, actual.jugador, mapa, sonambulo);
			if (cmpN2(actual, childTurnL) or cmpN2(childTurnL, actual))
			{
				if (!cerrados[childTurnL.jugador.f][childTurnL.jugador.c][childTurnL.jugador.brujula][l])
				{
					childTurnL.distancia += Distancia(actual, actTURN_L, mapa);
					childTurnL.secuencia.push_back(actTURN_L);
					abiertos.push(childTurnL);
				}
			}
			// Hijo actTURNR
			nodeN2 childTurnR = actual;
			childTurnR.jugador = apply(actTURN_R, actual.jugador, mapa, sonambulo);
			if (cmpN2(actual, childTurnR) or cmpN2(childTurnR, actual))
			{
				if (!cerrados[childTurnR.jugador.f][childTurnR.jugador.c][childTurnR.jugador.brujula][l])
				{
					childTurnR.distancia += Distancia(actual, actTURN_R, mapa);
					childTurnR.secuencia.push_back(actTURN_R);
					abiertos.push(childTurnR);
				}
			}
		}

		if (!fin and !abiertos.empty())
		{
			actual = abiertos.top();
			l = 0;
			if (actual.tieneZapatillas)
				l = 1;
			if (actual.tieneBikini)
				l = 2;
			while (!abiertos.empty() and cerrados[actual.jugador.f][actual.jugador.c][actual.jugador.brujula][l])
			{
				abiertos.pop();
				if (!abiertos.empty())
				{
					actual = abiertos.top();
					l = 0;
					if (actual.tieneZapatillas)
						l = 1;
					if (actual.tieneBikini)
						l = 2;
				}
			}
		}
	}
	return plan;
}

// Calcula el incremento de coste en batería por realizar una acción
int Distancia(const nodeN2 &origen, Action accion, const vector<vector<unsigned char>> &mapa)
{
	int distancia = 0;
	unsigned char casilla = mapa[origen.jugador.f][origen.jugador.c];
	switch (accion)
	{
	case actFORWARD:
		switch (casilla)
		{
		case 'A':
			distancia = (origen.tieneBikini) ? 10 : 100;
			break;
		case 'B':
			distancia = (origen.tieneZapatillas) ? 15 : 50;
			break;
		case 'T':
			distancia = 2;
			break;
		default:
			distancia = 1;
			break;
		}
		break;
	case actTURN_L:
		switch (casilla)
		{
		case 'A':
			distancia = (origen.tieneBikini) ? 5 : 25;
			break;
		case 'B':
			distancia = (origen.tieneZapatillas) ? 1 : 25;
			break;
		case 'T':
			distancia = 2;
			break;
		default:
			distancia = 1;
			break;
		}
		break;
	case actTURN_R:
		switch (casilla)
		{
		case 'A':
			distancia = (origen.tieneBikini) ? 5 : 25;
			break;
		case 'B':
			distancia = (origen.tieneZapatillas) ? 1 : 25;
			break;
		case 'T':
			distancia = 2;
			break;
		default:
			distancia = 1;
			break;
		}
		break;
	}
	return distancia;
}

// ==============================================
// NIVEL 3 ======================================
// ==============================================
int DistanciaManhattan(const ubicacion &origen, const ubicacion &destino)
{
	return abs(destino.f - origen.f) + abs(destino.c - origen.c);
}

int DistanciaChebyshev(const ubicacion &origen, const ubicacion &destino)
{
	return (max(abs(origen.f - destino.f), abs(origen.c - destino.c)));
}

int DistanciaChebyshevMejorada(const nodeN3 &origen, const ubicacion &destino, const unsigned char &casillaOrigenSon, const unsigned char &casillaOrigenJug)
{
	int distancia = DistanciaChebyshev(destino, origen.st.sonambulo);
	// Añadimos distancia manhatan entre jugador y objetivo - 7
	distancia += max(DistanciaManhattan(origen.st.jugador, destino) - 7, 0);
	// Y la distancia manhattan entre jugador y sonámbulo - 7
	distancia += max(DistanciaManhattan(origen.st.jugador, origen.st.sonambulo) - 7, 0);
	if (distancia > 0)
	{
		// Si no estamos en el final, como mínimo deberá haber un actSON_FORWARD más para salir de esa casilla
		// De esta forma, sumando más en las más costosas, ayudamos al algoritmo a encontrar la solución óptima haciendo que
		// expanda las más costosas las últimas, pero si el limite de la distancia cartesiana era mayor nos quedamos con ese
		switch (casillaOrigenSon)
		{
		case 'A':
			// Si es agua, lo único que reduciría el mínimo coste sería que el sonámbulo tenga bikini
			distancia = max((origen.tieneBikiniSon) ? 10 : 100, distancia);
			break;
		case 'B':
			// Si es bosque, solo tener zapatillas le reduce el coste de actFORWARD
			distancia = max((origen.tieneZapatillasSon) ? 15 : 50, distancia);
			break;
		case 'T':
			// En tierra siempre cuesta 2,
			distancia = max(2, distancia);
			break;
		default:
			distancia = distancia;
			break;
		}
		// También podemos asegurar, que si en un cuadrado de 7x7 alrededor del jugador (con este en el centro), no está el objetivo, entonces,
		// como mínimo, el jugador  también deberá hacer un actFORWARD
		if (DistanciaManhattan(origen.st.jugador, destino) > 8)
		{
			switch (casillaOrigenJug)
			{
			case 'A':
				// Si es agua, lo único que reduciría el mínimo coste sería que el sonámbulo tenga bikini
				distancia = max((origen.tieneBikini) ? 10 : 100, distancia);
				break;
			case 'B':
				// Si es bosque, solo tener zapatillas le reduce el coste de actFORWARD
				distancia = max((origen.tieneZapatillas) ? 15 : 50, distancia);
				break;
			case 'T':
				// En tierra siempre cuesta 2 más
				distancia = max(2, distancia);
				break;
			default:
				distancia = distancia;
				break;
			}
		}
	}
	return distancia;
}

// Similar al del nivel 2, calcula el incremento en batería de una acción
int Distancia(const nodeN3 &origen, Action accion, const vector<vector<unsigned char>> &mapa, bool sonambulo)
{
	// Añado aquí también el coste para las casillas desconocidas que se usará en el nivel 4
	// no habrá conflicto con el nivel3 porque simplemente no usará esos cases
	int distancia = 0;
	unsigned char casilla = mapa[origen.st.jugador.f][origen.st.jugador.c];
	if (sonambulo)
		casilla = mapa[origen.st.sonambulo.f][origen.st.sonambulo.c];
	switch (accion)
	{
	case actFORWARD:
		switch (casilla)
		{
		case 'A':
			distancia = (origen.tieneBikini) ? 10 : 100;
			break;
		case 'B':
			distancia = (origen.tieneZapatillas) ? 15 : 50;
			break;
		case 'T':
			distancia = 2;
			break;
		case '?':
			distancia = 5;
			break;
		default:
			distancia = 1;
			break;
		}
		break;
	case actTURN_L:
		switch (casilla)
		{
		case 'A':
			distancia = (origen.tieneBikini) ? 5 : 25;
			break;
		case 'B':
			distancia = (origen.tieneZapatillas) ? 1 : 25;
			break;
		case 'T':
			distancia = 2;
			break;
		case '?':
			distancia = 3;
			break;
		default:
			distancia = 1;
			break;
		}
		break;
	case actTURN_R:
		switch (casilla)
		{
		case 'A':
			distancia = (origen.tieneBikini) ? 5 : 25;
			break;
		case 'B':
			distancia = (origen.tieneZapatillas) ? 1 : 25;
			break;
		case 'T':
			distancia = 2;
			break;
		case '?':
			distancia = 3;
			break;
		default:
			distancia = 1;
			break;
		}
		break;
	case actSON_FORWARD:
		switch (casilla)
		{
		case 'A':
			distancia = (origen.tieneBikiniSon) ? 10 : 100;
			break;
		case 'B':
			distancia = (origen.tieneZapatillasSon) ? 15 : 50;
			break;
		case '?':
			distancia = 5;
			break;
		case 'T':
			distancia = 2;
			break;
		default:
			distancia = 1;
			break;
		}
		break;
	case actSON_TURN_SL:
		switch (casilla)
		{
		case 'A':
			distancia = (origen.tieneBikiniSon) ? 2 : 7;
			break;
		case 'B':
			distancia = (origen.tieneZapatillasSon) ? 1 : 3;
			break;
		case '?':
			distancia = 3;
			break;
		default:
			distancia = 1;
			break;
		}
		break;
	case actSON_TURN_SR:
		switch (casilla)
		{
		case 'A':
			distancia = (origen.tieneBikiniSon) ? 2 : 7;
			break;
		case 'B':
			distancia = (origen.tieneZapatillasSon) ? 1 : 3;
			break;
		case '?':
			distancia = 3;
			break;
		default:
			distancia = 1;
			break;
		}
		break;
	}
	return distancia;
}

bool cmpN3(nodeN3 a, nodeN3 n)
{
	stateN0 st = a.st;
	if (st.jugador.f < n.st.jugador.f)
		return true;
	else if (st.jugador.f == n.st.jugador.f and st.jugador.c < n.st.jugador.c)
		return true;
	else if (st.jugador.f == n.st.jugador.f and st.jugador.c == n.st.jugador.c and st.jugador.brujula < n.st.jugador.brujula)
		return true;
	else if (st.jugador.f == n.st.jugador.f and st.jugador.c == n.st.jugador.c and st.jugador.brujula == n.st.jugador.brujula and
			 st.sonambulo.f < n.st.sonambulo.f)
		return true;
	else if (st.jugador.f == n.st.jugador.f and st.jugador.c == n.st.jugador.c and st.jugador.brujula == n.st.jugador.brujula and
			 st.sonambulo.f == n.st.sonambulo.f and st.sonambulo.c < n.st.sonambulo.c)
		return true;
	else if (st.jugador.f == n.st.jugador.f and st.jugador.c == n.st.jugador.c and st.jugador.brujula == n.st.jugador.brujula and
			 st.sonambulo.f == n.st.sonambulo.f and st.sonambulo.c == n.st.sonambulo.c and st.sonambulo.brujula < n.st.sonambulo.brujula)
		return true;
	else if (st.jugador.f == n.st.jugador.f and st.jugador.c == n.st.jugador.c and st.jugador.brujula == n.st.jugador.brujula and
			 st.sonambulo.f == n.st.sonambulo.f and st.sonambulo.c == n.st.sonambulo.c and st.sonambulo.brujula == n.st.sonambulo.brujula and
			 a.tieneBikini < n.tieneBikini)
		return true;
	else if (st.jugador.f == n.st.jugador.f and st.jugador.c == n.st.jugador.c and st.jugador.brujula == n.st.jugador.brujula and
			 st.sonambulo.f == n.st.sonambulo.f and st.sonambulo.c == n.st.sonambulo.c and st.sonambulo.brujula == n.st.sonambulo.brujula and
			 a.tieneBikini == n.tieneBikini and a.tieneZapatillas < n.tieneZapatillas)
		return true;
	else if (st.jugador.f == n.st.jugador.f and st.jugador.c == n.st.jugador.c and st.jugador.brujula == n.st.jugador.brujula and
			 st.sonambulo.f == n.st.sonambulo.f and st.sonambulo.c == n.st.sonambulo.c and st.sonambulo.brujula == n.st.sonambulo.brujula and
			 a.tieneBikini == n.tieneBikini and a.tieneZapatillas == n.tieneZapatillas and a.tieneBikiniSon < n.tieneBikiniSon)
		return true;
	else if (st.jugador.f == n.st.jugador.f and st.jugador.c == n.st.jugador.c and st.jugador.brujula == n.st.jugador.brujula and
			 st.sonambulo.f == n.st.sonambulo.f and st.sonambulo.c == n.st.sonambulo.c and st.sonambulo.brujula == n.st.sonambulo.brujula and
			 a.tieneBikini == n.tieneBikini and a.tieneZapatillas == n.tieneZapatillas and a.tieneBikiniSon == n.tieneBikiniSon and a.tieneZapatillasSon < n.tieneZapatillasSon)
		return true;
	else
		return false;
}

list<Action> AStar(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa, const vector<vector<pair<int, int>>> &casillasTerreno)
{
	list<Action> plan;
	set<nodeN3, decltype(cmpN3) *> cerrados(cmpN3);

	int open = 0;
	int closed = 0;

	priority_queue<nodeN3> abiertos;

	nodeN3 actual;
	actual.g = 0;
	actual.st = inicio;
	actual.h = DistanciaChebyshevMejorada(actual, final, mapa[actual.st.sonambulo.f][actual.st.sonambulo.c], mapa[actual.st.jugador.f][actual.st.jugador.c]);
	actual.tieneBikini = actual.tieneZapatillas = actual.tieneBikiniSon = actual.tieneZapatillasSon = false;
	bool fin = (actual.st.sonambulo.f == final.f and actual.st.sonambulo.c == final.c);
	abiertos.push(actual);
	while (!fin)
	{
		actual = abiertos.top();
		abiertos.pop();
		// OBJETOS JUGADOR
		if (mapa[actual.st.jugador.f][actual.st.jugador.c] == 'K')
		{
			actual.tieneBikini = true;
			actual.tieneZapatillas = false;
		}
		if (mapa[actual.st.jugador.f][actual.st.jugador.c] == 'D')
		{
			actual.tieneZapatillas = true;
			actual.tieneBikini = false;
		}
		// Es indiferente que el jugador vea o no al sonambulo para que este coja los objetos, ya que
		// mientras el jugador no lo vea no va a poder "usarlos" por lo que da igual
		if (mapa[actual.st.sonambulo.f][actual.st.sonambulo.c] == 'K')
		{
			actual.tieneBikiniSon = true;
			actual.tieneZapatillasSon = false;
		}
		if (mapa[actual.st.sonambulo.f][actual.st.sonambulo.c] == 'D')
		{
			actual.tieneZapatillasSon = true;
			actual.tieneBikiniSon = false;
		}

		fin = (actual.st.sonambulo.f == final.f and actual.st.sonambulo.c == final.c);
		if (fin)
		{
			plan = actual.secuencia;
			cout << "Plan encontrado: ";
			for (auto it = plan.begin(); it != plan.end(); ++it)
				cout << *it << ", ";
			cout << endl;
		}
		// GENERA HIJOS
		else
		{
			if (EsVisible(actual.st, casillasTerreno))
			{
				// Hijo actSON_FORWARD
				nodeN3 childForwardSon = actual;
				childForwardSon.st = apply(actSON_FORWARD, childForwardSon.st, mapa);
				childForwardSon.g = actual.g + Distancia(actual, actSON_FORWARD, mapa, true);
				childForwardSon.h = DistanciaChebyshevMejorada(childForwardSon, final, mapa[childForwardSon.st.sonambulo.f][childForwardSon.st.sonambulo.c], mapa[childForwardSon.st.jugador.f][childForwardSon.st.jugador.c]);
				childForwardSon.secuencia.push_back(actSON_FORWARD);
				// Nos ahorramos una búsqueda si la acción no es posible
				if (cmpN3(actual, childForwardSon) or cmpN3(childForwardSon, actual))
				{
					if (cerrados.find(childForwardSon) == cerrados.end())
					{
						abiertos.push(childForwardSon);
					}
				}
				// Hijo actSON_TURN_SL
				nodeN3 childTurnLSon = actual;
				childTurnLSon.st = apply(actSON_TURN_SL, childTurnLSon.st, mapa);
				childTurnLSon.g = actual.g + Distancia(actual, actSON_TURN_SL, mapa, true);
				childTurnLSon.h = DistanciaChebyshevMejorada(childTurnLSon, final, mapa[childTurnLSon.st.sonambulo.f][childTurnLSon.st.sonambulo.c], mapa[childTurnLSon.st.jugador.f][childTurnLSon.st.jugador.c]);
				childTurnLSon.secuencia.push_back(actSON_TURN_SL);
				if (cmpN3(actual, childTurnLSon) or cmpN3(childTurnLSon, actual))
				{
					if (cerrados.find(childTurnLSon) == cerrados.end())
					{
						abiertos.push(childTurnLSon);
					}
				}
				// Hijo actSON_TURN_SR
				nodeN3 childTurnRSon = actual;
				childTurnRSon.st = apply(actSON_TURN_SR, childTurnRSon.st, mapa);
				childTurnRSon.g = actual.g + Distancia(actual, actSON_TURN_SR, mapa, true);
				childTurnRSon.h = DistanciaChebyshevMejorada(childTurnRSon, final, mapa[childTurnRSon.st.sonambulo.f][childTurnRSon.st.sonambulo.c], mapa[childTurnRSon.st.jugador.f][childTurnRSon.st.jugador.c]);
				childTurnRSon.secuencia.push_back(actSON_TURN_SR);
				if (cmpN3(actual, childTurnRSon) or cmpN3(childTurnRSon, actual))
				{
					if (cerrados.find(childTurnRSon) == cerrados.end())
					{
						abiertos.push(childTurnRSon);
					}
				}
			}
			// Hijo act_FORWARD
			nodeN3 childForward = actual;
			childForward.st = apply(actFORWARD, childForward.st, mapa);
			childForward.g = actual.g + Distancia(actual, actFORWARD, mapa, false);
			childForward.h = DistanciaChebyshevMejorada(childForward, final, mapa[childForward.st.sonambulo.f][childForward.st.sonambulo.c], mapa[childForward.st.jugador.f][childForward.st.jugador.c]);
			childForward.secuencia.push_back(actFORWARD);
			if (cmpN3(actual, childForward) or cmpN3(childForward, actual))
			{
				if (cerrados.find(childForward) == cerrados.end())
				{
					abiertos.push(childForward);
				}
			}
			// Hijo actTURN_R
			nodeN3 childTurnR = actual;
			childTurnR.st = apply(actTURN_R, childTurnR.st, mapa);
			childTurnR.g = actual.g + Distancia(actual, actTURN_R, mapa, false);
			childTurnR.h = actual.h;
			childTurnR.secuencia.push_back(actTURN_R);
			if (cmpN3(actual, childTurnR) or cmpN3(childTurnR, actual))
			{
				if (cerrados.find(childTurnR) == cerrados.end())
				{
					abiertos.push(childTurnR);
				}
			}
			// Hijo actTURN_L
			nodeN3 childTurnL = actual;
			childTurnL.st = apply(actTURN_L, childTurnL.st, mapa);
			childTurnL.g = actual.g + Distancia(actual, actTURN_L, mapa, false);
			childTurnL.h = actual.h;
			childTurnL.secuencia.push_back(actTURN_L);
			if (cmpN3(actual, childTurnL) or cmpN3(childTurnL, actual))
			{
				if (cerrados.find(childTurnL) == cerrados.end())
				{
					abiertos.push(childTurnL);
				}
			}
		}
		cerrados.insert(actual);
		// Evitamos expandir nodos repetidos
		if (!fin and !abiertos.empty())
		{
			actual = abiertos.top();
			while (!abiertos.empty() and cerrados.find(actual) != cerrados.end())
			{
				abiertos.pop();
				if (!abiertos.empty())
					actual = abiertos.top();
			}
		}
	}
	return plan;
}

// ==============================================
// NIVEL 4 ======================================
// ==============================================
stateN4 UpdateState(const stateN4 &st, const Action accion)
{
	stateN4 result = st;
	int a;
	switch (accion)
	{
	case actFORWARD:
		switch (st.jugador.brujula)
		{
		case norte:
			result.jugador.f--;
			break;
		case noreste:
			result.jugador.f--;
			result.jugador.c++;
			break;
		case este:
			result.jugador.c++;
			break;
		case sureste:
			result.jugador.f--;
			result.jugador.c++;
			break;
		case sur:
			result.jugador.f++;
			break;
		case suroeste:
			result.jugador.f++;
			result.jugador.c--;
			break;
		case oeste:
			result.jugador.c--;
			break;
		case noroeste:
			result.jugador.c--;
			result.jugador.f--;
			break;
		}
		break;
	case actSON_FORWARD:
		switch (st.sonambulo.brujula)
		{
		case norte:
			result.sonambulo.f--;
			break;
		case noreste:
			result.sonambulo.f--;
			result.sonambulo.c++;
			break;
		case este:
			result.sonambulo.c++;
			break;
		case sureste:
			result.sonambulo.f++;
			result.sonambulo.c++;
			break;
		case sur:
			result.sonambulo.f++;
			break;
		case suroeste:
			result.sonambulo.f++;
			result.sonambulo.c--;
			break;
		case oeste:
			result.sonambulo.c--;
			break;
		case noroeste:
			result.sonambulo.c--;
			result.sonambulo.f--;
			break;
		}
		break;
	case actTURN_L:
		a = st.jugador.brujula;
		a = (a + 6) % 8;
		result.jugador.brujula = static_cast<Orientacion>(a);
		break;
	case actTURN_R:
		a = st.jugador.brujula;
		a = (a + 2) % 8;
		result.jugador.brujula = static_cast<Orientacion>(a);
		break;
	case actSON_TURN_SL:
		a = st.sonambulo.brujula;
		a = (a + 7) % 8;
		result.sonambulo.brujula = static_cast<Orientacion>(a);
		break;
	case actSON_TURN_SR:
		a = st.sonambulo.brujula;
		a = (a + 1) % 8;
		result.sonambulo.brujula = static_cast<Orientacion>(a);
		break;
	}
	return result;
}

void rellenaMapa(const vector<unsigned char> &terreno, vector<vector<unsigned char>> &mapa, const ubicacion &st, const vector<vector<pair<int, int>>> &casillasTerreno)
{
	int i0 = st.f, j0 = st.c;
	mapa[i0][j0] = terreno[0];
	for (int i = 1; i < terreno.size(); ++i)
	{
		int fil = i0 + casillasTerreno[static_cast<int>(st.brujula)][i].first;
		int col = j0 + casillasTerreno[static_cast<int>(st.brujula)][i].second;
		mapa[fil][col] = terreno[i];
	}
}

void ComportamientoJugador::Nivel4(Sensores sensores)
{
	// Si colisionamos o nos coge un lobo ya no estamos bien situados
	if (sensores.colision or sensores.reset)
		current_stateN4.bienSituado = false;
	// Si no estamos bien situados, lo primero es ubicarse
	if (!current_stateN4.bienSituado)
	{
		plan.push_back(actWHEREIS);
		current_stateN4.bienSituado = true;
		current_stateN4.ubicandose = true;
	}
	else
	{
		// Cuando hagamos actWHEREIS nos ubicamos
		if (sensores.posF != -1)
		{
			current_stateN4.ubicandose = false;
			current_stateN4.bienSituado = true;
			current_stateN4.jugador.f = sensores.posF;
			current_stateN4.jugador.c = sensores.posC;
			current_stateN4.jugador.brujula = sensores.sentido;
			current_stateN4.sonambulo.f = sensores.SONposF;
			current_stateN4.sonambulo.c = sensores.SONposC;
			current_stateN4.sonambulo.brujula = sensores.SONsentido;
		}
		// Rellenamos el mapa con la información actual de la visión
		rellenaMapa(sensores.terreno, mapaResultado, current_stateN4.jugador, casillasTerreno);
		// Nos guardamos información de los objetos
		if (sensores.terreno[0] == 'K')
		{
			current_stateN4.tieneBikini = true;
			current_stateN4.tieneZapatillas = false;
		}
		if (sensores.terreno[0] == 'D')
		{
			current_stateN4.tieneBikini = false;
			current_stateN4.tieneZapatillas = true;
		}
		if (mapaResultado[current_stateN4.sonambulo.f][current_stateN4.sonambulo.c] == 'K')
		{
			current_stateN4.tieneBikiniSon = true;
			current_stateN4.tieneZapatillasSon = false;
		}
		if (mapaResultado[current_stateN4.sonambulo.f][current_stateN4.sonambulo.c] == 'D')
		{
			current_stateN4.tieneBikiniSon = false;
			current_stateN4.tieneZapatillasSon = true;
		}
		stateN0 inicio;
		inicio.jugador = current_stateN4.jugador;
		inicio.sonambulo = current_stateN4.sonambulo;

		// Medimos cuánto mapa conocemos alrededor del objetivo
		int descubiertas = 0;
		int contadas = 0;
		for (int i = max(0, static_cast<int>(goal.f - mapaResultado.size() / 5)); i < min(mapaResultado.size(), goal.f + mapaResultado.size() / 5); ++i)
		{
			for (int j = max(0, static_cast<int>(goal.c - mapaResultado.size() / 5)); j < min(mapaResultado.size(), goal.c + mapaResultado.size() / 5); ++j)
			{
				++contadas;
				if (mapaResultado[i][j] != '?')
					++descubiertas;
			}
		}
		float perc = (float)(descubiertas) / (contadas);
		// Distinguimos si el sonámbulo está muy lejos, porque entonces la búsqueda tarda demasiado y es más conveniente ir solo con el jugador
		if ((DistanciaChebyshev(current_stateN4.sonambulo, goal) > 10))
		{
			// Si conocemos menos de la mitad de la zona definida alrededor del objetivo, la solución
			// obtenida por Dijkstra va a ser mala por falta de información, por lo que usamos anchura
			if (perc < 0.5)
			{
				cout << "AnchuraSoloJugador" << endl;
				plan = AnchuraSoloJugador(inicio, goal, mapaResultado);
			}
			else
			{
				cout << "DijkstraJugador" << endl;
				plan = DijkstraSoloJugador(inicio, goal, mapaResultado);
			}
		}
		// Si estamos cerca del objetivo con el sonámbulo, lo buscamos con anchura
		// Por problemas de eficiencia con AStar este algoritmo no se usaen este nivel
		else
		{
			cout << "AnchuraSonambulo" << endl;
			plan = AnchuraSonambulo(inicio, goal, mapaResultado, casillasTerreno);
		}
	}
}

// ==============================================
// AUXILIARES ======================================
// ==============================================
void print_queue(priority_queue<nodeN2> q)
{
	while (!q.empty())
	{
		cout << q.top().jugador << " Distancia: " << q.top().distancia << endl;
		q.pop();
	}
	cout << endl;
}
void print_queue(priority_queue<nodeN3> q, vector<vector<unsigned char>> &mapaConPlan)
{
	while (!q.empty())
	{
		// cout << q.top().st << ((q.top().tieneBikini) ? "BIK ": "--- ") << ((q.top().tieneZapatillas) ? "ZAP // " : "--- // ") <<
		// (q.top().tieneBikiniSon ? "BIK ": "--- ") << (q.top().tieneZapatillasSon ? "ZAP" : "---")<<endl<<
		// "Distancia: " << q.top().g+q.top().h << endl;
		mapaConPlan[q.top().st.sonambulo.f][q.top().st.sonambulo.c] = 1;
		q.pop();
	}
	cout << endl;
}
void AnularMatriz(vector<vector<unsigned char>> &matriz)
{
	for (int i = 0; i < matriz.size(); i++)
		for (int j = 0; j < matriz[0].size(); j++)
			matriz[i][j] = 0;
}
void ComportamientoJugador::VisualizaPlan(const stateN0 &st, const list<Action> &plan)
{
	AnularMatriz(mapaConPlan);
	stateN0 cst = st;
	auto it = plan.begin();
	while (it != plan.end())
	{
		switch (*it)
		{
		case actFORWARD:
			cst.jugador = NextCasilla(cst.jugador);
			mapaConPlan[cst.jugador.f][cst.jugador.c] = 1;
			break;
		case actTURN_R:
			cst.jugador.brujula = (Orientacion)((cst.jugador.brujula + 2) % 8);
			break;
		case actTURN_L:
			cst.jugador.brujula = (Orientacion)((cst.jugador.brujula + 6) % 8);
			break;
		case actSON_FORWARD:
			cst.sonambulo = NextCasilla(cst.sonambulo);
			mapaConPlan[cst.sonambulo.f][cst.sonambulo.c] = 2;
			break;
		case actSON_TURN_SR:
			cst.sonambulo.brujula = (Orientacion)((cst.sonambulo.brujula + 1) % 8);
			break;
		case actSON_TURN_SL:
			cst.sonambulo.brujula = (Orientacion)((cst.sonambulo.brujula + 7) % 8);
			break;
		}
		++it;
	}
}
stateN0 apply(Action action, const stateN0 &current_state, const vector<vector<unsigned char>> &mapa)
{
	stateN0 stResult = current_state;
	ubicacion sigUbicacion;
	switch (action)
	{
	case actFORWARD:
		sigUbicacion = NextCasilla(current_state.jugador);
		if (CasillaTransitable(sigUbicacion, mapa) and
			!(sigUbicacion.f == current_state.sonambulo.f and sigUbicacion.c == current_state.sonambulo.c))
		{
			stResult.jugador = sigUbicacion;
		}
		break;
	case actTURN_L:
		stResult.jugador.brujula = static_cast<Orientacion>((stResult.jugador.brujula + 6) % 8);
		break;
		break;
	case actTURN_R:
		stResult.jugador.brujula = static_cast<Orientacion>((stResult.jugador.brujula + 2) % 8);
		break;
	case actSON_FORWARD:
		sigUbicacion = NextCasilla(current_state.sonambulo);
		if (CasillaTransitable(sigUbicacion, mapa) and
			!(sigUbicacion.f == current_state.jugador.f and sigUbicacion.c == current_state.jugador.c))
			stResult.sonambulo = sigUbicacion;
		break;
	case actSON_TURN_SL:
		stResult.sonambulo.brujula = static_cast<Orientacion>((stResult.sonambulo.brujula + 7) % 8);
		break;
	case actSON_TURN_SR:
		stResult.sonambulo.brujula = static_cast<Orientacion>((stResult.sonambulo.brujula + 1) % 8);
		break;
	}
	return stResult;
}
bool CasillaTransitable(const ubicacion &x, const vector<vector<unsigned char>> &mapa)
{
	return (mapa[x.f][x.c] != 'P' and mapa[x.f][x.c] != 'M');
}
ubicacion NextCasilla(const ubicacion &pos)
{
	ubicacion next = pos;
	switch (next.brujula)
	{
	case norte:
		next.f--;
		break;
	case noreste:
		next.f--;
		next.c++;
		break;
	case este:
		next.c++;
		break;
	case sureste:
		next.f++;
		next.c++;
		break;
	case sur:
		next.f++;
		break;
	case suroeste:
		next.f++;
		next.c--;
		break;
	case oeste:
		next.c--;
		break;
	case noroeste:
		next.c--;
		next.f--;
		break;
	default:
		break;
	}
	return next;
}
int ComportamientoJugador::interact(Action accion, int valor)
{
	return false;
}
ostream &operator<<(ostream &out, const ubicacion &x)
{
	out << "Fila / Col / Orientacion: " << x.f << " / " << x.c << " / " << x.brujula << endl;
	return out;
}
ostream &operator<<(ostream &out, const stateN0 &x)
{
	out << "Ubicación jugador:" << endl;
	out << x.jugador;
	out << "Ubicación sonámbulo:" << endl;
	out << x.sonambulo;
	return out;
};