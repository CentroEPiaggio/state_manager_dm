#ifndef DRAW_STATE_MACHINE_HPP
#define DRAW_STATE_MACHINE_HPP

#include <map>
#include <vector>
#include <unistd.h>
#include <lemon/dijkstra.h>
#include <lemon/concepts/maps.h>
#include <lemon/graph_to_eps.h>
#include <lemon/smart_graph.h>

struct stat;
namespace dual_manipulation{
    namespace state_manager{

        template <class state_type, class transition_type>
        class draw_state_machine
        {
            lemon::SmartDigraph graph;
            lemon::SmartDigraph::ArcMap<int> length;
            lemon::SmartDigraph::NodeMap<state_type> states_ids;
            lemon::SmartDigraph::ArcMap<transition_type> arcs_ids;
            lemon::SmartDigraph::NodeMap<lemon::dim2::Point<int>> coords;

        public:
            draw_state_machine():length(graph),states_ids(graph),coords(graph),arcs_ids(graph)
            {
            }
            std::map<state_type,std::map<transition_type,state_type>> transition_table;
            void insert(std::vector< std::tuple< state_type, transition_type, state_type > > table)
            {
                for (auto row:table)
                    insert(row);
            }
            
            void insert(std::tuple< state_type, transition_type, state_type > row)
            {
                transition_table[std::get<0>(row)][std::get<1>(row)]=std::get<2>(row);
            }
            
            void calculateForces(double& xvel,double& yvel, lemon::SmartDigraph::NodeIt& nsource)
            {
                xvel = 0;
                yvel = 0;
                // Sum up all forces pushing this item away
                for (lemon::SmartDigraph::NodeIt ntarget(graph); ntarget!=lemon::INVALID; ++ntarget) //for all the sources
                {
                    double dx = coords[nsource][0]-coords[ntarget][0];
                    double dy = coords[nsource][1]-coords[ntarget][1];
                    double l = 2.0 * (dx * dx + dy * dy);
                    if (l > 0) {
                        xvel += (dx * 150.0) / l;
                        yvel += (dy * 150.0) / l;
                    }
                }
                // Now subtract all forces pulling items together
                double weight = (countArcs(graph)+1) * 10;
                for (lemon::SmartDigraph::OutArcIt a(graph, nsource); a!=lemon::INVALID; ++a)
                {
                    auto delta=coords[graph.target(a)]-coords[nsource];
                    xvel -= delta[0] / weight;
                    yvel -= delta[1] / weight;
                }
                for (lemon::SmartDigraph::InArcIt a(graph, nsource); a!=lemon::INVALID; ++a)
                {
                    auto delta=coords[graph.source(a)]-coords[nsource];
                    xvel -= delta[0] / weight;
                    yvel -= delta[1] / weight;
                }
                if (abs(xvel) < 0.1)
                    xvel=0;
                if (abs(yvel) < 0.1)
                    yvel = 0;
            }

            void draw_on_file(std::string filename)
            {
                graph.clear();
                int x=0;
                for ( auto state:transition_table )
                {
                    x++;
                    lemon::SmartDigraph::Node n = graph.addNode();
                    states_ids[n] = state.first;
                    coords[n].x=rand()%100-50;
                    coords[n].y=rand()%100-50;
                }

                for (lemon::SmartDigraph::NodeIt nsource(graph); nsource!=lemon::INVALID; ++nsource) //for all the sources
                {
                    for (lemon::SmartDigraph::NodeIt ntarget(graph); ntarget!=lemon::INVALID; ++ntarget) //for all the targets
                    {
                        for (auto transition:transition_table[states_ids[nsource]])
                        {
                            if (transition.second==states_ids[ntarget])
                            {
                                lemon::SmartDigraph::Arc a=graph.addArc ( nsource,ntarget );
                                arcs_ids[a]=transition.first;
                                length[a]=1;
                            }
                        }
                    }
                }

                //repulsive approach
                for (int i=0;i<50;i++)
                {
                    double xvel=0;
                    double yvel=0;
                    for (lemon::SmartDigraph::NodeIt nsource(graph); nsource!=lemon::INVALID; ++nsource) //for all the sources
                    {
                        calculateForces(xvel,yvel,nsource);
                        coords[nsource][0]+=xvel;
                        coords[nsource][1]+=yvel;
                        if (coords[nsource][0]>50) coords[nsource][0]=50;
                        if (coords[nsource][0]<-50) coords[nsource][0]=-50;
                        if (coords[nsource][1]>50) coords[nsource][1]=50;
                        if (coords[nsource][1]<-50) coords[nsource][1]=-50;
                    }
                }
                lemon::graphToEps<lemon::SmartDigraph> ( graph,filename.c_str() ).
                coords ( coords ).
                //    nodeColors ( composeMap ( p,ncolors ) ).
                //    arcColors ( composeMap ( p,acolors ) ).
                nodeTexts ( states_ids ).
                nodeTextSize (  3 ).
                nodeScale ( 0.02 ).
                arcWidthScale ( 0.0014 ).
                drawArrows ( true ).
                arrowWidth ( 3 ).
                arcTexts(arcs_ids).
                arcTextSize(3).
                arrowLength ( 5 ).
                //     enableParallel ( true ).
                enableParallel().parArcDist(5).
                distantColorNodeTexts().
                run();
            }
        };

    }
}
#endif // DRAW_STATE_MACHINE_HPP